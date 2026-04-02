#include <cstring>

#include "gps.hpp"

static constexpr uint8_t UBX_SYNC_1 = 0xB5;
static constexpr uint8_t UBX_SYNC_2 = 0x62;
static constexpr uint8_t UBX_MESSAGE_CLASS = 0x01;
static constexpr uint8_t UBC_MESSAGE_ID = 0x11;

GPS::GPS(UART_HandleTypeDef* huart) : huart(huart) {}

bool GPS::init() {
    HAL_StatusTypeDef success = HAL_UARTEx_ReceiveToIdle_DMA(
		huart,
		rxBuffer,
		MAX_NMEA_DATA_LENGTH
    );

    __HAL_DMA_DISABLE_IT(huart->hdmarx, DMA_IT_HT);

    HAL_StatusTypeDef messagesuccess = enableMessage(0x01, 0x11); //enable ubx velecef messages
    
    return (success == HAL_OK) & (messagesuccess == HAL_OK);
}

HAL_StatusTypeDef GPS::enableMessage(uint8_t msgClass, uint8_t msgId) {
    uint8_t cfgMsg[] = {
        0xB5, 0x62,         //sync chars
        0x06, 0x01,         //class and id
        0x03, 0x00,         //length
        msgClass, msgId,
        0x01,               //rate
        0x00, 0x00          //placeholder for checksum
    };

    if(sendUBX(cfgMsg, sizeof(cfgMsg))) {
        return HAL_OK;
    } else {
        return HAL_ERROR;
    }
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

GpsData_t GPS::readData() {
    __HAL_UART_DISABLE_IT(huart, UART_IT_IDLE);
    __HAL_DMA_DISABLE_IT(huart->hdmarx, DMA_IT_TC);

    bool success = parseRMC() && parseGGA() && parseUBX();
    tempData.isNew = success;
    validData = tempData;

    validData.isNew = false;

   __HAL_UART_ENABLE_IT(huart, UART_IT_IDLE);
   __HAL_DMA_ENABLE_IT(huart->hdmarx, DMA_IT_TC);

    return tempData;
}

void GPS::rxCallback(uint16_t size) {
    memcpy(processBuffer, rxBuffer, MAX_NMEA_DATA_LENGTH);
    HAL_UARTEx_ReceiveToIdle_DMA(
		huart,
		rxBuffer,
		size
    );
    processBufferEnd = processBuffer + size;
    __HAL_DMA_DISABLE_IT(huart->hdmarx, DMA_IT_HT);
}

UART_HandleTypeDef* GPS::getHUART() {
    return huart;
}

bool GPS::parseUBX() {
    uint16_t idx = 0;
    if (!incrementProcessBufferIndex(idx, 1)) return false;

    // Find sync
    while (!(processBuffer[idx - 1] == UBX_SYNC_1 && processBuffer[idx] == UBX_SYNC_2)) {
        if (!incrementProcessBufferIndex(idx, 1)) return false;
    }
    // Check message class and ID
    if (processBuffer[idx+2] != UBX_MESSAGE_CLASS || processBuffer[idx+3] != UBC_MESSAGE_ID) {
        return false;
    }

    // Payload
    if (!incrementProcessBufferIndex(idx, 6)) return false;

    if (!incrementProcessBufferIndex(idx, 4)) return false;

    if(getVx(idx) == false) {
    	return false;
    }

    if(getVy(idx) == false) {
    	return false;
    }

    if (getVz(idx) == false) {
    	return false;
    }

    return true;
}


bool GPS::parseRMC() {
    uint16_t idx = 0;
    if (!incrementProcessBufferIndex(idx, 2)) return false;
    while (!(processBuffer[idx - 2] == 'R' && processBuffer[idx - 1] == 'M' && processBuffer[idx] == 'C')) {
        if (!incrementProcessBufferIndex(idx, 1)) return false;
    }

    if (!incrementProcessBufferIndex(idx, 4)) return false;

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

bool GPS::parseGGA() {
    uint16_t idx = 0;
    if (!incrementProcessBufferIndex(idx, 2)) return false;
    while (!(processBuffer[idx - 2] == 'G' && processBuffer[idx - 1] == 'G' && processBuffer[idx] == 'A')) {
        if (!incrementProcessBufferIndex(idx, 1)) return false;
    }
    if (!incrementProcessBufferIndex(idx, 4)) return false; // Skip to data

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


bool GPS::getVx(uint16_t &idx) {
    if (!incrementProcessBufferIndex(idx, 3)) return false;
    int32_t ecefVX = (int32_t)((uint32_t)processBuffer[idx - 3] |
                               ((uint32_t)processBuffer[idx - 2] << 8) |
                               ((uint32_t)processBuffer[idx - 1] << 16) |
                               ((uint32_t)processBuffer[idx] << 24));

    tempData.vx = ecefVX / 100.0f;
    return true;
}

bool GPS::getVy(uint16_t &idx) {
    if (!incrementProcessBufferIndex(idx, 3)) return false;
    int32_t ecefVY = (int32_t)((uint32_t)processBuffer[idx - 3] |
                               ((uint32_t)processBuffer[idx - 2] << 8) |
                               ((uint32_t)processBuffer[idx - 1] << 16) |
                               ((uint32_t)processBuffer[idx] << 24));

    tempData.vy = ecefVY / 100.0f;
    return true;
}

bool GPS::getVz(uint16_t &idx) {
    if (!incrementProcessBufferIndex(idx, 3)) return false;
    int32_t ecefVZ = (int32_t)((uint32_t)processBuffer[idx - 3] |
                               ((uint32_t)processBuffer[idx - 2] << 8) |
                               ((uint32_t)processBuffer[idx - 1] << 16) |
                               ((uint32_t)processBuffer[idx] << 24));

    tempData.vz = ecefVZ / 100.0f;
    return true;
}

bool GPS::incrementProcessBufferIndex(uint16_t &idx, uint16_t increment) {
    if (processBuffer + idx >= processBufferEnd) return false;
    idx += increment;
    return true;
}
