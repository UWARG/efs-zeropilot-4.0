#include <cstring>

#include "gps.hpp"

static constexpr uint8_t UBX_SYNC_1 = 0xB5;
static constexpr uint8_t UBX_SYNC_2 = 0x62;

static constexpr uint8_t UBX_MESSAGE_CLASS_NAV = 0x01;
static constexpr uint8_t UBX_MESSAGE_CLASS_ACK = 0x05;
static constexpr uint8_t UBX_MESSAGE_CLASS_CFG = 0x06;
static constexpr uint8_t UBX_MESSAGE_CLASS_NMEA = 0xF0;

static constexpr uint8_t UBX_MESSAGE_ID_VELECEF = 0x11;
static constexpr uint8_t UBX_MESSAGE_ID_PVT = 0x07;
static constexpr uint8_t UBX_MESSAGE_ID_ACK_ACK = 0x01;
static constexpr uint8_t UBX_MESSAGE_ID_CFG_VALSET = 0x8A;
static constexpr uint8_t UBX_MESSAGE_ID_CFG_MSG = 0x01;
static constexpr uint8_t UBX_MESSAGE_ID_CFG_RATE = 0x08;

static constexpr uint8_t CFG_LAYER_RAM = 0x01;

static constexpr uint16_t UBX_HEADER_LEN = 6; // Sync chars(2), class(1), ID(1), payload length(2)
static constexpr uint16_t UBX_CHECKSUM_LEN = 2;
static constexpr uint8_t UBX_ACK_FRAME_LEN = 8; // ACK frame length after its sync chars: class(1), ID(1), length(2), payload(2), checksum(2)
static constexpr uint8_t UBX_ACK_PAYLOAD_LEN = 2;
static constexpr uint32_t UBX_ACK_TIMEOUT_MS = 250;

static constexpr uint8_t PVT_EXPECTED_LEN = 92;
static constexpr uint8_t VELECEF_EXPECTED_LEN = 20;

static constexpr uint8_t MESSAGE_RATE_DISABLED = 0;
static constexpr uint8_t MESSAGE_RATE_EVERY_SOLUTION = 1;

// Config key for nominal time between GNSS measurements, used with CFG-VALSET
static constexpr uint32_t CFG_KEY_RATE_MEAS = 0x30210001;
// Config key for ratio of number of measurements to number of navigation solutions, used with CFG-VALSET
static constexpr uint32_t CFG_KEY_RATE_NAV  = 0x30210002;
// Config keys for the message output rates on UART1, used with CFG-VALSET
static constexpr uint32_t CFG_KEY_MSGOUT_UBX_NAV_PVT_UART1 = 0x20910007;
static constexpr uint32_t CFG_KEY_MSGOUT_NMEA_UART1[] = {
    0x209100BB, // GGA
    0x209100CA, // GLL
    0x209100C0, // GSA
    0x209100C5, // GSV
    0x209100AC, // RMC
    0x209100B1  // VTG
};

// NMEA msg IDs used with CFG-MSG
static constexpr uint8_t NMEA_SENTENCE_IDS[] = {
    0x00, // GGA
    0x01, // GLL
    0x02, // GSA
    0x03, // GSV
    0x04, // RMC
    0x05  // VTG
};

static constexpr uint8_t NMEA_SENTENCE_START = '$';
static constexpr uint8_t NMEA_SENTENCE_END = '\n';
static constexpr uint8_t NMEA_CHECKSUM_DELIMITER = '*';
static constexpr uint16_t NMEA_SENTENCE_TYPE_OFFSET = 3; // A sentence opens with '$' followed by a two character talker ID
static constexpr uint16_t NMEA_SENTENCE_TYPE_LEN = 3;
static constexpr uint16_t NMEA_SENTENCE_TYPE_SKIP = 4; // The sentence type plus the comma that closes the field

static constexpr uint8_t FIX_TYPE_2D = 2;
static constexpr uint8_t FIX_TYPE_3D = 3;

// Time Validity flags: bit0 validDate, bit1 validTime, bit2 fullyResolved
static constexpr uint8_t PVT_VALID_TIME_MASK = 0b00000111;
// Fix status flags: bit0 gnssFixOK
static constexpr uint8_t PVT_GNSS_FIX_OK_MASK = 0b00000001;

// Read a signed 32 bit int in little endian
static int32_t readInt32LE(const uint8_t *buf, uint16_t idx);

// Convert ASCII hex to numeric value
static int8_t hexValue(uint8_t c);

GPS::GPS(UART_HandleTypeDef* huart) :
    huart(huart) {}

bool GPS::init() {
    SET_BIT(huart->Instance->CR3, USART_CR3_OVRDIS);

    // Configure before starting the DMA receive, waitForAck() is polling
    protocol = configureUBX() ? UBX : NMEA;

    return restartDMA() == HAL_OK;
}

/*
Enable NAV-PVT before disabling NMEA. If this is not acknowledged the receiver keeps emitting its NMEA messages
Switch the gps to UBX NAV-PVT only. Returns false if the receiver does not support UBX and 
leaves its NMEA output untouched so readData() can still parse it
*/
bool GPS::configureUBX() {
    // Try CFG-VALSET config msg, prefered for chips M9/M10
    if (configValset(CFG_KEY_MSGOUT_UBX_NAV_PVT_UART1, MESSAGE_RATE_EVERY_SOLUTION)) {
        // PVT is confirmed, so NMEA is redundant. Disable NMEA messages
        for (uint32_t key : CFG_KEY_MSGOUT_NMEA_UART1) {
            configValset(key, MESSAGE_RATE_DISABLED);
        }
        // Configure GPS ODR to be 5Hz
        configValset(CFG_KEY_RATE_MEAS, 200);
        configValset(CFG_KEY_RATE_NAV, 1);
        return true;
    }

    // GPS is M8 or older, retry over legacy CFG-MSG
    if (setMessageRate(UBX_MESSAGE_CLASS_NAV, UBX_MESSAGE_ID_PVT, MESSAGE_RATE_EVERY_SOLUTION)) {
        // PVT is confirmed, so NMEA is redundant. Disable NMEA messages
        for (uint8_t sentenceId : NMEA_SENTENCE_IDS) {
            setMessageRate(UBX_MESSAGE_CLASS_NMEA, sentenceId, MESSAGE_RATE_DISABLED);
        }
        setRate(200, 1);
        return true;
    }

    return false;
}

/*
Send config message via CFG-MSG for M8 or older
Rate 0 disables the message, 1 emits it on every navigation solution
*/
bool GPS::setMessageRate(uint8_t msgClass, uint8_t msgId, uint8_t rate) {
    uint8_t cfgMsg[] = {
        UBX_SYNC_1, UBX_SYNC_2,
        UBX_MESSAGE_CLASS_CFG, UBX_MESSAGE_ID_CFG_MSG,
        0x03, 0x00, // Length
        msgClass, msgId, rate,
        0x00, 0x00 // Placeholder for checksum
    };
    if (!sendUBX(cfgMsg, sizeof(cfgMsg))) return false;
    return waitForAck(UBX_MESSAGE_CLASS_CFG, UBX_MESSAGE_ID_CFG_MSG);
}

// Legacy CFG-RATE for M8 and older. measRate in ms, navRate in cycles.
bool GPS::setRate(uint16_t measRateMs, uint16_t navRate) {
    uint8_t cfgRate[] = {
        UBX_SYNC_1, UBX_SYNC_2,
        UBX_MESSAGE_CLASS_CFG, UBX_MESSAGE_ID_CFG_RATE,
        0x06, 0x00, // Length
        (uint8_t)(measRateMs), (uint8_t)(measRateMs >> 8), // The elapsed time between GNSS measurements
        (uint8_t)(navRate), (uint8_t)(navRate >> 8), // The ratio between the number of measurements and the number of navigation solutions
        0x01, 0x00, // timeRef = 1 (GPS time)
        0x00, 0x00  // Checksum placeholder
    };
    if (!sendUBX(cfgRate, sizeof(cfgRate))) return false;
    return waitForAck(UBX_MESSAGE_CLASS_CFG, UBX_MESSAGE_ID_CFG_RATE);
}

/*
Send config message via UBX-CFG-VALSET for M9/M10
Rate 0 disables the message, 1 emits it on every navigation solution
*/
bool GPS::configValset(uint32_t key, uint32_t value) {
    // The value width (1/2/4 bytes) is encoded in bits 30:28 of the key
    uint8_t valueLen;
    switch ((key >> 28) & 0x07) {
        case 0x2: valueLen = 1; break; // U1 or bool
        case 0x3: valueLen = 2; break; // U2
        case 0x4: valueLen = 4; break; // U4
        default:  return false; // Unsupported width
    }

    // Header(6) + cfg preamble(4) + key(4) + value(<=4) + checksum(2)
    uint8_t msg[UBX_HEADER_LEN + 4 + 4 + 4 + UBX_CHECKSUM_LEN] = {};
    uint16_t i = 0;
    msg[i++] = UBX_SYNC_1;
    msg[i++] = UBX_SYNC_2;
    msg[i++] = UBX_MESSAGE_CLASS_CFG;
    msg[i++] = UBX_MESSAGE_ID_CFG_VALSET;
    msg[i++] = (uint8_t)(4 + 4 + valueLen); // Payload length low byte, fixed overhead(version, layers, reserved) + key + value
    msg[i++] = 0x00; // Payload length high byte
    msg[i++] = 0x00; // Version (transactionless, apply config immediately)
    msg[i++] = CFG_LAYER_RAM; // Layers
    msg[i++] = 0x00; // Reserved
    msg[i++] = 0x00; // Reserved
    msg[i++] = (uint8_t)(key);
    msg[i++] = (uint8_t)(key >> 8);
    msg[i++] = (uint8_t)(key >> 16);
    msg[i++] = (uint8_t)(key >> 24);
    for (uint8_t b = 0; b < valueLen; b++) {
        msg[i++] = (uint8_t)(value >> (8 * b)); // Little-endian
    }
    msg[i++] = 0x00; // Checksum placeholder
    msg[i++] = 0x00;

    if (!sendUBX(msg, i)) return false;
    return waitForAck(UBX_MESSAGE_CLASS_CFG, UBX_MESSAGE_ID_CFG_VALSET);
}

void GPS::calcChecksum(uint8_t *msg, uint16_t len) {
    uint8_t ckA = 0, ckB = 0;
    for (int i = 2; i < len - 2; i++) {
        ckA += msg[i];
        ckB += ckA;
    }
    msg[len - 2] = ckA;
    msg[len - 1] = ckB;
}

bool GPS::sendUBX(uint8_t *msg, uint16_t len) {
    calcChecksum(msg, len);
    return (HAL_UART_Transmit(huart, msg, len, HAL_MAX_DELAY) == HAL_OK);
}

/*
Scans the incoming msgs in polling for an ACK msgClass and msgId. 
A gps that does not speak UBX never ACKs and timeout tells us to fall back to NMEA
*/
bool GPS::waitForAck(uint8_t msgClass, uint8_t msgId) {
    const uint32_t deadline = HAL_GetTick() + UBX_ACK_TIMEOUT_MS;

    uint8_t byte = 0;
    while (receiveByte(byte, deadline)) {
        if (byte != UBX_SYNC_1) continue;
        if (!receiveByte(byte, deadline) || byte != UBX_SYNC_2) continue;

        uint8_t body[UBX_ACK_FRAME_LEN] = {0};
        bool success = true;
        for (uint8_t i = 0; i < UBX_ACK_FRAME_LEN && success; i++) {
            success = receiveByte(body[i], deadline);
        }
        if (!success) break;

        // Some other UBX frame arrived first, keep scanning
        if (body[0] != UBX_MESSAGE_CLASS_ACK) continue;
        if (body[2] != UBX_ACK_PAYLOAD_LEN || body[3] != 0) continue; // Check payload length

        // Check the payload msgClass and msgId being acknowledged
        if (body[4] != msgClass || body[5] != msgId) continue;

        // An ACK-NAK means the receiver refused the request
        return body[1] == UBX_MESSAGE_ID_ACK_ACK;
    }

    return false;
}

bool GPS::receiveByte(uint8_t &byte, uint32_t deadline) {
    int32_t remainingTick = (int32_t)(deadline - HAL_GetTick());
    if (remainingTick <= 0) return false;

    return HAL_UART_Receive(huart, &byte, 1, (uint32_t)remainingTick) == HAL_OK;
}

GpsData_t GPS::readData() {
    // When nothing new has arrived since the last call, mark it as old and don't reparse the data
    if (!dataReady) {
        tempData.isNew = false;
        return tempData;
    }
    dataReady = false;

    // Claim the buffer so rxCallback cannot overwrite it mid-parse
    parsingData = true;

    bool success = false;
    uint16_t idx = 0;
    const uint16_t len = processBufferLen();
    while (idx < len) {
        if (processBuffer[idx] == UBX_SYNC_1) {
            success |= consumeUBX(idx);
        } else if (processBuffer[idx] == NMEA_SENTENCE_START) {
            success |= consumeNMEA(idx);
        } else {
            idx++;
        }
    }

    tempData.isNew = success;

    parsingData = false;
    return tempData;
}

void GPS::rxCallback(uint16_t size) {
    // Refresh the parse buffer only when readData() is not mid-parse
    if (!parsingData && size > 0) {
        memcpy((uint8_t*)processBuffer, (uint8_t*)rxBuffer, size);
        processBufferEnd = (uint8_t*)processBuffer + size;
        dataReady = true;
    }

    restartDMA();
}

HAL_StatusTypeDef GPS::restartDMA() {
    HAL_StatusTypeDef status = HAL_UARTEx_ReceiveToIdle_DMA(
        huart,
        (uint8_t*)rxBuffer,
        MAX_NMEA_DATA_LENGTH
    );
    __HAL_DMA_DISABLE_IT(huart->hdmarx, DMA_IT_HT);
    return status;
}

UART_HandleTypeDef* GPS::getHuart() {
    return huart;
}

GpsProtocol_t GPS::getProtocol() {
    return protocol;
}

uint16_t GPS::processBufferLen() {
    return (uint16_t)(processBufferEnd - (uint8_t*)processBuffer);
}

// idx enters on the sync char 1(0xB5) and leaves past the end of the frame, whether or not the frame parsed, so that readData() always makes progress
bool GPS::consumeUBX(uint16_t &idx) {
    const uint16_t start = idx;
    const uint16_t len = processBufferLen();

    // Step over it, not long enough to form a header or not a real UBX frame
    if (start + UBX_HEADER_LEN > len || processBuffer[start + 1] != UBX_SYNC_2) {
        idx = start + 1;
        return false;
    }

    uint8_t msgClass = processBuffer[start + 2];
    uint8_t msgId = processBuffer[start + 3];
    uint16_t payloadLen = ((uint16_t)processBuffer[start + 5] << 8) | ((uint16_t)processBuffer[start + 4]);

    // Check if the full frame is actually inside the buffer
    uint32_t frameLen = (uint32_t)UBX_HEADER_LEN + payloadLen + UBX_CHECKSUM_LEN;
    if (start + frameLen > len) {
        idx = len; // The rest has not arrived, drop the msg
        return false;
    }

    idx = start + (uint16_t)frameLen;

    if (!verifyChecksumUBX(start, (uint16_t)frameLen)) return false;

    if (msgClass != UBX_MESSAGE_CLASS_NAV) return false;

    // The parse functions expect to start on the length field
    uint16_t parseStart = start + 4;
    switch (msgId) {
        case UBX_MESSAGE_ID_VELECEF:
            return parseVELECEF(parseStart);
        case UBX_MESSAGE_ID_PVT:
            return parsePVT(parseStart);
        default:
            return false; // Drop all other msgs
    }
}

// idx enters on the '$' and leaves past \n, whether or not the sentence parsed, so readData() always makes progress
bool GPS::consumeNMEA(uint16_t &idx) {
    const uint16_t start = idx;
    const uint16_t len = processBufferLen();

    // Find the end index of the msg
    uint16_t end = start;
    while ((end < len) && (processBuffer[end] != NMEA_SENTENCE_END)) end++;
    if (end == len) {
        idx = len; // The full msg didnt arrive, drop the msg
        return false;
    }

    idx = end + 1;

    if (!verifyChecksumNMEA(start, end)) return false;

    uint16_t parseStart = start + NMEA_SENTENCE_TYPE_OFFSET;
    if (parseStart + NMEA_SENTENCE_TYPE_LEN > end) return false;

    if (matchesSentenceType(parseStart, "RMC")) return parseRMC(parseStart);
    if (matchesSentenceType(parseStart, "GGA")) return parseGGA(parseStart);

    return false;
}

bool GPS::matchesSentenceType(uint16_t idx, const char *sentenceType) {
    for (uint16_t i = 0; i < NMEA_SENTENCE_TYPE_LEN; i++) {
        if (processBuffer[idx + i] != (uint8_t)sentenceType[i]) return false;
    }
    return true;
}

bool GPS::verifyChecksumUBX(uint16_t start, uint16_t frameLen) {
    uint8_t ckA = 0, ckB = 0;
    // Covers everything between the sync chars and the checksum itself
    for (uint16_t i = start + 2; i < start + frameLen - UBX_CHECKSUM_LEN; i++) {
        ckA += processBuffer[i];
        ckB += ckA;
    }

    return ckA == processBuffer[start + frameLen - 2] && ckB == processBuffer[start + frameLen - 1];
}

bool GPS::verifyChecksumNMEA(uint16_t start, uint16_t end) {
    // Checksum is the XOR of the bytes between '$' and '*'
    uint16_t idx = start + 1;
    uint8_t checksum = 0;

    // Calculate expected checksum
    while ((idx < end) && (processBuffer[idx] != NMEA_CHECKSUM_DELIMITER)) {
        checksum ^= processBuffer[idx];
        idx++;
    }

    // Parse received checksum 
    if (idx + 2 >= end) return false;
    int8_t high = hexValue(processBuffer[idx + 1]);
    int8_t low = hexValue(processBuffer[idx + 2]);
    if (high < 0 || low < 0) return false;

    return checksum == (uint8_t)((high << 4) | low);
}

// idx enters on the 'R' of "RMC"
bool GPS::parseRMC(uint16_t &idx) {
    if (!incrementProcessBufferIndex(idx, NMEA_SENTENCE_TYPE_SKIP)) return false;

    // Check if data exists
    if (processBuffer[idx] == ',') {
        return false;
    }

    if (getTimeRMC(idx) == false) {
        return false;
    }

    // Skip to status
    while (processBuffer[idx] != ',') if (!incrementProcessBufferIndex(idx, 1)) return false;;

    // Begin status
    if (!incrementProcessBufferIndex(idx, 1)) return false;

    // Check if data valid
    if (processBuffer[idx] == 'V') return false;
    // End status

    if (!incrementProcessBufferIndex(idx, 2)) return false;

    if (getLatitudeRMC(idx) == false) {
        return false;
    }

    if (!incrementProcessBufferIndex(idx, 2)) return false;

    if (getLongitudeRMC(idx) == false) {
        return false;
    }

    if (!incrementProcessBufferIndex(idx, 2)) return false;

    if (getSpeedRMC(idx) == false) {
        return false;
    }

    while (processBuffer[idx] != ',') if (!incrementProcessBufferIndex(idx, 1)) return false;;
    if (!incrementProcessBufferIndex(idx, 1)) return false;

    if (getTrackAngleRMC(idx) == false) {
        return false;
    }

    while (processBuffer[idx] != ',') if (!incrementProcessBufferIndex(idx, 1)) return false;
    while (processBuffer[idx] == ',') if (!incrementProcessBufferIndex(idx, 1)) return false;

    if (getDateRMC(idx) == false) {
        return false;
    }

    return true;
}

// idx enters on the first 'G' of "GGA"
bool GPS::parseGGA(uint16_t &idx) {
    if (!incrementProcessBufferIndex(idx, NMEA_SENTENCE_TYPE_SKIP)) return false;

    // Check if data exists
    if (processBuffer[idx] == ',') {
        return false;
    }

    // Skip 7 sections of data
    for (int i = 0; i < 6; i++) {
        while (processBuffer[idx] != ',') if (!incrementProcessBufferIndex(idx, 1)) return false;
        if (!incrementProcessBufferIndex(idx, 1)) return false;
    }

    if (getNumSatellitesGGA(idx) == false) {
        return false;
    }

    for (int i = 0; i < 2; i++) {
    	while (processBuffer[idx] != ',') if (!incrementProcessBufferIndex(idx, 1)) return false;
        if (!incrementProcessBufferIndex(idx, 1)) return false;
    }

    if(getAltitudeGGA(idx) == false) {
    	return false;
    }

    return true;
}

bool GPS::parseVELECEF(uint16_t &idx) {
    if (getLenUBX(idx) != VELECEF_EXPECTED_LEN) return false;

    // Skip iTOW field
    if (!incrementProcessBufferIndex(idx, 4)) return false;

    if (!getVxVELECEF(idx)) return false;

    if (!getVyVELECEF(idx)) return false;

    if (!getVzVELECEF(idx)) return false;

    return true;
}

bool GPS::parsePVT(uint16_t &idx) {
    // Length field
    if (getLenUBX(idx) != PVT_EXPECTED_LEN) return false;

    // Skip iTOW field
    if (!incrementProcessBufferIndex(idx, 4)) return false;

    // Consume year(2), month(1), day(1), hour(1), min(1), sec(1), valid(1)
    if (!incrementProcessBufferIndex(idx, 8)) return false;
    bool validTimeData = (processBuffer[idx - 1] & PVT_VALID_TIME_MASK) == PVT_VALID_TIME_MASK;
    if (validTimeData) {
        tempData.time.year = ((uint16_t)processBuffer[idx - 7] << 8) | ((uint16_t)processBuffer[idx - 8]);
        tempData.time.month = processBuffer[idx - 6];
        tempData.time.day = processBuffer[idx - 5];
        tempData.time.hour = processBuffer[idx - 4];
        tempData.time.minute = processBuffer[idx - 3];
        tempData.time.second = processBuffer[idx - 2];
    }

    // Skip tAcc(4 bytes) and nano(4 bytes) fields
    if (!incrementProcessBufferIndex(idx, 8)) return false;

    // Consume fixType(1), flags(1), flags2(1), numSV(1), lon(4), lat(4), height(4), hMSL(4)
    if (!incrementProcessBufferIndex(idx, 20)) return false;
    uint8_t fixType = processBuffer[idx - 20];
    bool gnssFixOK = processBuffer[idx - 19] & PVT_GNSS_FIX_OK_MASK;
    if (!gnssFixOK || (fixType != FIX_TYPE_2D && fixType != FIX_TYPE_3D)) {
        return false;
    }
    tempData.numSatellites = processBuffer[idx - 17];
    tempData.longitude = readInt32LE((uint8_t*)processBuffer, idx - 16) * 1e-7f; // 1e-7 deg to deg
    tempData.latitude = readInt32LE((uint8_t*)processBuffer, idx - 12) * 1e-7f; // 1e-7 deg to deg
    // hMSL is height above mean sea level, 2D fix carries no usable altitude
    tempData.altitude = (fixType == FIX_TYPE_3D)
        ? readInt32LE((uint8_t*)processBuffer, idx - 4) / 1000.0f // mm to m
        : INVALID_ALTITUDE;

    // Skip hAcc(4) and vAcc(4) fields
    if (!incrementProcessBufferIndex(idx, 8)) return false;

    // Consume the NED velocities velN(4), velE(4), velD(4)
    if (!incrementProcessBufferIndex(idx, 12)) return false;
    tempData.vx = readInt32LE((uint8_t*)processBuffer, idx - 12) / 1000.0f; // mm/s to m/s
    tempData.vy = readInt32LE((uint8_t*)processBuffer, idx - 8) / 1000.0f;
    tempData.vz = readInt32LE((uint8_t*)processBuffer, idx - 4) / 1000.0f;

    // Get gSpeed field
    if (!incrementProcessBufferIndex(idx, 4)) return false;
    tempData.groundSpeed = readInt32LE((uint8_t*)processBuffer, idx - 4) / 10.0f; // mm/s to cm/s

    // Get headMot field
    if (!incrementProcessBufferIndex(idx, 4)) return false;
    tempData.trackAngle = readInt32LE((uint8_t*)processBuffer, idx - 4) * 1e-5f; // 1e-5 deg to deg

    return true;
}

uint16_t GPS::getLenUBX(uint16_t &idx) {
    if (!incrementProcessBufferIndex(idx, 2)) return 0;
    return ((uint16_t)processBuffer[idx - 1] << 8) | ((uint16_t)processBuffer[idx - 2]);  
}

bool GPS::getTimeRMC(uint16_t &idx) {
    uint8_t hour = (processBuffer[idx] - '0') * 10 + (processBuffer[idx + 1] - '0');
    if (!incrementProcessBufferIndex(idx, 2)) return false;
    uint8_t minute = (processBuffer[idx] - '0') * 10 + (processBuffer[idx + 1] - '0');
    if (!incrementProcessBufferIndex(idx, 2)) return false;
    uint8_t second = (processBuffer[idx] - '0') * 10 + (processBuffer[idx + 1] - '0');

    tempData.time.hour = hour;
    tempData.time.minute = minute;
    tempData.time.second = second;

    return true;
}

bool GPS::getLatitudeRMC(uint16_t &idx) {
    float lat = 0;
    for (int i = 0; i < 2; i++) {
        lat *= 10;
        lat += ((float)(processBuffer[idx] - '0'));
        if (!incrementProcessBufferIndex(idx, 1)) return false;
    }

    float lat_minutes = 0;
    while (processBuffer[idx] != '.') {
        lat_minutes *= 10;
        lat_minutes += ((float)(processBuffer[idx] - '0'));
        if (!incrementProcessBufferIndex(idx, 1)) return false;
    }
    if (!incrementProcessBufferIndex(idx, 1)) return false;; // Skip decimal char

    // Including two digits of minutes
    uint32_t mult = 10;
    while (processBuffer[idx] != ',' && mult <= DECIMAL_PRECISION) {
        lat_minutes += ((float)(processBuffer[idx] - '0')) / mult;
        if (!incrementProcessBufferIndex(idx, 1)) return false;
        mult *= 10;
    }

    lat += lat_minutes/60;

    tempData.latitude = lat;

    // Skip to NS char indicator
    while (processBuffer[idx] != ',') if (!incrementProcessBufferIndex(idx, 1)) return false;
    if (!incrementProcessBufferIndex(idx, 1)) return false; // Skip over comma
    tempData.latitude *= (processBuffer[idx] == 'N') ? 1 : -1;

    return true;
}

bool GPS::getLongitudeRMC(uint16_t &idx) {
    float lon = 0;
    for (int i = 0; i < 3; i++) {
        lon *= 10;
        lon += ((float)(processBuffer[idx] - '0'));
        if (!incrementProcessBufferIndex(idx, 1)) return false;
    }

    float lon_minutes = 0;
    while (processBuffer[idx] != '.') {
        lon_minutes *= 10;
        lon_minutes += ((float)(processBuffer[idx] - '0'));
        if (!incrementProcessBufferIndex(idx, 1)) return false;
    }

    if (!incrementProcessBufferIndex(idx, 1)) return false; // Skip decimal char

    // Including two digits of minutes
    uint32_t mult = 10;
    while (processBuffer[idx] != ',' && mult <= DECIMAL_PRECISION) {
        lon_minutes += ((float)(processBuffer[idx] - '0')) / mult;
        if (!incrementProcessBufferIndex(idx, 1)) return false;
        mult *= 10;
    }

    lon += lon_minutes / 60;

    tempData.longitude = lon;
    while (processBuffer[idx] != ',') if (!incrementProcessBufferIndex(idx, 1)) return false;
    if (!incrementProcessBufferIndex(idx, 1)) return false;
    tempData.longitude *= (processBuffer[idx] == 'E') ? 1 : -1;

    return true;
}

bool GPS::getSpeedRMC(uint16_t &idx) {
    float spd = 0;
    while (processBuffer[idx] != '.') {
        spd *= 10;
        spd += processBuffer[idx] - '0';
        if (!incrementProcessBufferIndex(idx, 1)) return false;
    }
    if (!incrementProcessBufferIndex(idx, 1)) return false; // Decimal char
    uint32_t mult = 10;
    while (processBuffer[idx] != ',' && mult <= DECIMAL_PRECISION) {
        spd += ((float)(processBuffer[idx] - '0')) / mult;
        if (!incrementProcessBufferIndex(idx, 1)) return false;
        mult *= 10;
    }

    tempData.groundSpeed = spd * 51.4444; // Convert from kt to cm/s

    return true;
}

bool GPS::getTrackAngleRMC(uint16_t &idx) {
    float cog = 0;
    // Check if cog was calculated
    if (processBuffer[idx] != ',') {
        while (processBuffer[idx] != '.') {
            cog *= 10;
            cog += processBuffer[idx] - '0';
            if (!incrementProcessBufferIndex(idx, 1)) return false;
        }
        if (!incrementProcessBufferIndex(idx, 1)) return false; // Decimal char
        uint32_t mult = 10;
        while (processBuffer[idx] != ',' && mult <= DECIMAL_PRECISION) {
            cog += ((float)(processBuffer[idx] - '0')) / mult;
            if (!incrementProcessBufferIndex(idx, 1)) return false;
            mult *= 10;
        }
    }
    else {
        cog = INVALID_TRACK_ANGLE;
    }

    tempData.trackAngle = cog;

    return true;
}

bool GPS::getDateRMC(uint16_t &idx) {
    int day = (processBuffer[idx] - '0') * 10 + processBuffer[idx + 1] - '0';
    if (!incrementProcessBufferIndex(idx, 2)) return false;
    int month = (processBuffer[idx] - '0') * 10 + processBuffer[idx + 1] - '0';
    if (!incrementProcessBufferIndex(idx, 2)) return false;
    int year = (processBuffer[idx] - '0') * 10 + processBuffer[idx + 1] - '0';

    tempData.time.day = day;
    tempData.time.month= month;
    tempData.time.year = year;

    return true;
}

bool GPS::getNumSatellitesGGA(uint16_t &idx) {
    int numSats = 0;
    while (processBuffer[idx] != ',') {
        numSats *= 10;
        numSats += processBuffer[idx] - '0';
        if (!incrementProcessBufferIndex(idx, 1)) return false;
    }

    tempData.numSatellites = numSats;

    return true;
}

bool GPS::getAltitudeGGA(uint16_t &idx) {
    float altitude = 0;
    while (processBuffer[idx] != '.') {
        altitude *= 10;
        altitude += processBuffer[idx] - '0';
        if (!incrementProcessBufferIndex(idx, 1)) return false;
    }
    if (!incrementProcessBufferIndex(idx, 1)) return false; // Decimal char
    uint32_t mult = 10;
    while (processBuffer[idx] != ',' && mult <= DECIMAL_PRECISION) {
        altitude += ((float)(processBuffer[idx] - '0')) / mult;
        if (!incrementProcessBufferIndex(idx, 1)) return false;
        mult *= 10;
    }

    tempData.altitude = altitude;
    return true;
}

bool GPS::getVxVELECEF(uint16_t &idx) {
    if (!incrementProcessBufferIndex(idx, 4)) return false;
    int32_t ecefVX = readInt32LE((uint8_t*)processBuffer, idx - 4);

    tempData.vx = ecefVX / 100.0f; // cm/s to m/s
    return true;
}

bool GPS::getVyVELECEF(uint16_t &idx) {
    if (!incrementProcessBufferIndex(idx, 4)) return false;
    int32_t ecefVY = readInt32LE((uint8_t*)processBuffer, idx - 4);

    tempData.vy = ecefVY / 100.0f; // cm/s to m/s
    return true;
}

bool GPS::getVzVELECEF(uint16_t &idx) {
    if (!incrementProcessBufferIndex(idx, 4)) return false;
    int32_t ecefVZ = readInt32LE((uint8_t*)processBuffer, idx - 4);

    tempData.vz = ecefVZ / 100.0f; // cm/s to m/s
    return true;
}

bool GPS::incrementProcessBufferIndex(uint16_t &idx, uint16_t increment) {
    if (processBuffer + idx + increment >= processBufferEnd) return false;
    idx += increment;
    return true;
}

static int32_t readInt32LE(const uint8_t *buf, uint16_t idx) {
    return (int32_t)((uint32_t)buf[idx] |
                    ((uint32_t)buf[idx + 1] << 8) |
                    ((uint32_t)buf[idx + 2] << 16) |
                    ((uint32_t)buf[idx + 3] << 24));
}

static int8_t hexValue(uint8_t c) {
    if (c >= '0' && c <= '9') return c - '0';
    if (c >= 'A' && c <= 'F') return c - 'A' + 10;
    if (c >= 'a' && c <= 'f') return c - 'a' + 10;
    return -1; // Character is not a hex digit
}
