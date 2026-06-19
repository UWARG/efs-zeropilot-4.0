#include <cstring>

#include "gps.hpp"

GPS::GPS(UART_HandleTypeDef* huart) : huart(huart) {}

UART_HandleTypeDef* GPS::getHUART() {
    return huart;
}

ZP_ERROR_e GPS::init() {
    ZP_ERROR_e result = ZP_ERROR_OK;

    // Start UART DMA
    HAL_StatusTypeDef hal_status = HAL_UARTEx_ReceiveToIdle_DMA(
        huart,
        rxBuffer,
        MAX_NMEA_DATA_LENGTH
    );

    if (hal_status != HAL_OK) {
        result |= ZP_ERROR_FAIL;
    }

    // Enable IDLE interrupt
    __HAL_UART_ENABLE_IT(huart, UART_IT_IDLE);

    // Enable UBX velocity messages
    result |= enableMessage(0x01, 0x11);

    return result;
}

ZP_ERROR_e GPS::enableMessage(uint8_t msgClass, uint8_t msgId) {
    uint8_t cfgMsg[] = {
        0xB5, 0x62,         //sync chars
        0x06, 0x01,         //class and id
        0x03, 0x00,         //length
        msgClass, msgId,
        0x01,               //rate
        0x00, 0x00          //placeholder for checksum
    };

    ZP_ERROR_e result = sendUBX(cfgMsg, sizeof(cfgMsg));
    return result;
}

ZP_ERROR_e GPS::calcChecksum(uint8_t *msg, uint16_t len) {
    if (msg == nullptr) {
        return ZP_ERROR_NULLPTR;
    }

    uint8_t ckA = 0, ckB = 0;
    for (int i = 2; i < len - 2; i++) {
        ckA += msg[i];
        ckB += ckA;
    }
    msg[len - 2] = ckA;
    msg[len - 1] = ckB;
    return ZP_ERROR_OK;
}

ZP_ERROR_e GPS::sendUBX(uint8_t *msg, uint16_t len) {
    ZP_ERROR_e result = calcChecksum(msg, len);
    if (result != ZP_ERROR_OK) {
        return result;
    }

    HAL_StatusTypeDef status = HAL_UART_Transmit(huart, msg, len, HAL_MAX_DELAY);
    if (status != HAL_OK) {
        return ZP_ERROR_FAIL;
    }

    return ZP_ERROR_OK;
}

ZP_ERROR_e GPS::readData(GpsData_t& out_data) {
    // Critical section: disable DMA to prevent buffer corruption during parsing
    __HAL_DMA_DISABLE_IT(huart->hdmarx, DMA_IT_TC);

    ZP_ERROR_e result = ZP_ERROR_OK;

    // Accumulate errors from each parsing step
    // Note: We use |= so we know exactly which parser failed if any return an error code
    result |= parseRMC();
    result |= parseGGA();
    result |= parseUBX();

    // Logic: If result is OK, update the data
    if (result == ZP_ERROR_OK) {
        tempData.isNew = true;
        validData = tempData;
        validData.isNew = false;
    }

    out_data = tempData;
    // Re-enable DMA
    __HAL_DMA_ENABLE_IT(huart->hdmarx, DMA_IT_TC);

    return result;
}

ZP_ERROR_e GPS::processGPSData() {
    memcpy(processBuffer, rxBuffer, MAX_NMEA_DATA_LENGTH);
    return ZP_ERROR_OK;
}

ZP_ERROR_e GPS::parseUBX() {
    int idx = 0;
    
    // Find Sync Chars
    while (!(processBuffer[idx] == 0xB5 && processBuffer[idx + 1] == 0x62)) {
        idx++;
        // Use ZP_ERROR_PARSE if we reach the end of the buffer without finding a message
        if (idx >= (MAX_NMEA_DATA_LENGTH - 1)) return ZP_ERROR_PARSE;
    }
    
    // Validate Class and ID
    if (processBuffer[idx + 2] != 0x01 || processBuffer[idx + 3] != 0x11) {
        return ZP_ERROR_UNSUPPORTED; // Message is valid UBX but not the type we want
    }

    // Move index to start of payload (Sync(2) + Class/ID(2) + Length(2))
    idx += 6; 
    
    // Skip iTOW (4 bytes)
    idx += 4;

    // Use | to accumulate result and return immediately if any step fails.
    // Assuming getVx, Vy, Vz now return ZP_ERROR_e.
    ZP_ERROR_e result = ZP_ERROR_OK;

    result |= getVx(idx);
    if (result != ZP_ERROR_OK) return result;
    idx += 4;

    result |= getVy(idx);
    if (result != ZP_ERROR_OK) return result;
    idx += 4;

    result |= getVz(idx);
    return result;
}


ZP_ERROR_e GPS::parseRMC() {
    int idx = 0;
    while (!(processBuffer[idx] == 'R' && processBuffer[idx+1] == 'M' && processBuffer[idx+2] == 'C')) {
        idx++;
        if (idx >= MAX_NMEA_DATA_LENGTH - 1) return ZP_ERROR_PARSE;
    }

    idx += 4;

    if (processBuffer[idx] == ',') return ZP_ERROR_PARSE;

    ZP_ERROR_e result = ZP_ERROR_OK;
    result |= getTimeRMC(idx);
    if (result != ZP_ERROR_OK) return result;

    while (processBuffer[idx] != ',') idx++;
    idx++;

    if (processBuffer[idx] == 'V') return ZP_ERROR_FAIL;

    idx += 2;
    result |= getLatitudeRMC(idx);
    idx += 2;
    result |= getLongitudeRMC(idx);
    idx += 2;
    result |= getSpeedRMC(idx);

    while (processBuffer[idx] != ',') idx++;
    idx++;

    result |= getTrackAngleRMC(idx);

    while (processBuffer[idx] != ',') idx++;
    while (processBuffer[idx] == ',') idx++;

    result |= getDateRMC(idx);

    return result;
}

ZP_ERROR_e GPS::parseGGA() {
    int idx = 0;
    while (!(processBuffer[idx] == 'G' && processBuffer[idx + 1] == 'G' && processBuffer[idx + 2] == 'A')) {
        idx++;
        if (idx >= MAX_NMEA_DATA_LENGTH - 1) return ZP_ERROR_PARSE;
    }
    idx += 4;

    if (processBuffer[idx] == ',') return ZP_ERROR_PARSE;

    for (int i = 0; i < 6; i++) {
        while (processBuffer[idx] != ',') idx++;
        idx++;
    }

    ZP_ERROR_e result = ZP_ERROR_OK;
    result |= getNumSatellitesGGA(idx);

    for (int i = 0; i < 2; i++) {
        while (processBuffer[idx] != ',') idx++;
        idx++;
    }

    result |= getAltitudeGGA(idx);

    return result;
}

ZP_ERROR_e GPS::getTimeRMC(int &idx) {
    if (idx + 6 >= MAX_NMEA_DATA_LENGTH) return ZP_ERROR_MEMORY_OVERFLOW;
    
    tempData.time.hour = (processBuffer[idx] - '0') * 10 + (processBuffer[idx + 1] - '0');
    idx += 2;
    tempData.time.minute = (processBuffer[idx] - '0') * 10 + (processBuffer[idx + 1] - '0');
    idx += 2;
    tempData.time.second = (processBuffer[idx] - '0') * 10 + (processBuffer[idx + 1] - '0');

    return ZP_ERROR_OK;
}

ZP_ERROR_e GPS::getLatitudeRMC(int &idx) {
    if (idx + 10 >= MAX_NMEA_DATA_LENGTH) return ZP_ERROR_MEMORY_OVERFLOW;
    
    float lat = 0;
    for (int i = 0; i < 2; i++, idx++) {
        lat = lat * 10.0f + (processBuffer[idx] - '0');
    }

    float lat_minutes = 0;
    while (processBuffer[idx] != '.') {
        lat_minutes = lat_minutes * 10.0f + (processBuffer[idx] - '0');
        idx++;
    }
    idx++;

    uint32_t mult = 10;
    while (processBuffer[idx] != ',' && mult <= DECIMAL_PRECISION) {
        lat_minutes += (processBuffer[idx] - '0') / (float)mult;
        idx++;
        mult *= 10;
    }

    tempData.latitude = lat + (lat_minutes / 60.0f);
    while (processBuffer[idx] != ',') idx++;
    idx++;
    if (processBuffer[idx] == 'S') tempData.latitude *= -1.0f;

    return ZP_ERROR_OK;
}

ZP_ERROR_e GPS::getLongitudeRMC(int &idx) {
    if (idx + 11 >= MAX_NMEA_DATA_LENGTH) return ZP_ERROR_MEMORY_OVERFLOW;

    float lon = 0;
    for (int i = 0; i < 3; i++, idx++) {
        lon = lon * 10.0f + (processBuffer[idx] - '0');
    }

    float lon_minutes = 0;
    while (processBuffer[idx] != '.') {
        lon_minutes = lon_minutes * 10.0f + (processBuffer[idx] - '0');
        idx++;
    }
    idx++;

    uint32_t mult = 10;
    while (processBuffer[idx] != ',' && mult <= DECIMAL_PRECISION) {
        lon_minutes += (processBuffer[idx] - '0') / (float)mult;
        idx++;
        mult *= 10;
    }

    tempData.longitude = lon + (lon_minutes / 60.0f);
    while (processBuffer[idx] != ',') idx++;
    idx++;
    if (processBuffer[idx] == 'W') tempData.longitude *= -1.0f;

    return ZP_ERROR_OK;
}

ZP_ERROR_e GPS::getSpeedRMC(int &idx) {
    if (idx >= MAX_NMEA_DATA_LENGTH) return ZP_ERROR_MEMORY_OVERFLOW;
    
    float spd = 0;
    while (processBuffer[idx] != '.' && processBuffer[idx] != ',') {
        spd = spd * 10.0f + (processBuffer[idx] - '0');
        idx++;
    }
    if (processBuffer[idx] == '.') {
        idx++;
        uint32_t mult = 10;
        while (processBuffer[idx] != ',' && mult <= DECIMAL_PRECISION) {
            spd += (processBuffer[idx] - '0') / (float)mult;
            idx++;
            mult *= 10;
        }
    }

    tempData.groundSpeed = spd * 51.4444f;
    return ZP_ERROR_OK;
}

ZP_ERROR_e GPS::getTrackAngleRMC(int &idx) {
    if (idx >= MAX_NMEA_DATA_LENGTH) return ZP_ERROR_MEMORY_OVERFLOW;
    
    if (processBuffer[idx] == ',') {
        tempData.trackAngle = INVALID_TRACK_ANGLE;
        return ZP_ERROR_OK;
    }

    float cog = 0;
    while (processBuffer[idx] != '.') {
        cog = cog * 10.0f + (processBuffer[idx] - '0');
        idx++;
    }
    idx++;
    uint32_t mult = 10;
    while (processBuffer[idx] != ',' && mult <= DECIMAL_PRECISION) {
        cog += (processBuffer[idx] - '0') / (float)mult;
        idx++;
        mult *= 10;
    }

    tempData.trackAngle = cog;
    return ZP_ERROR_OK;
}

ZP_ERROR_e GPS::getDateRMC(int &idx) {
    if (idx + 6 >= MAX_NMEA_DATA_LENGTH) return ZP_ERROR_MEMORY_OVERFLOW;
    
    tempData.time.day   = (processBuffer[idx] - '0') * 10 + (processBuffer[idx + 1] - '0');
    tempData.time.month = (processBuffer[idx + 2] - '0') * 10 + (processBuffer[idx + 3] - '0');
    tempData.time.year  = (processBuffer[idx + 4] - '0') * 10 + (processBuffer[idx + 5] - '0');
    idx += 6;

    return ZP_ERROR_OK;
}

ZP_ERROR_e GPS::getNumSatellitesGGA(int &idx) {
    if (idx >= MAX_NMEA_DATA_LENGTH) return ZP_ERROR_MEMORY_OVERFLOW;
    
    int numSats = 0;
    while (processBuffer[idx] != ',') {
        numSats = numSats * 10 + (processBuffer[idx] - '0');
        idx++;
    }
    tempData.numSatellites = numSats;
    return ZP_ERROR_OK;
}

ZP_ERROR_e GPS::getAltitudeGGA(int &idx) {
    if (idx >= MAX_NMEA_DATA_LENGTH) return ZP_ERROR_MEMORY_OVERFLOW;
    
    float altitude = 0;
    while (processBuffer[idx] != '.') {
        altitude = altitude * 10.0f + (processBuffer[idx] - '0');
        idx++;
    }
    idx++;
    uint32_t mult = 10;
    while (processBuffer[idx] != ',' && mult <= DECIMAL_PRECISION) {
        altitude += (processBuffer[idx] - '0') / (float)mult;
        idx++;
        mult *= 10;
    }
    tempData.altitude = altitude;
    return ZP_ERROR_OK;
}

ZP_ERROR_e GPS::getVx(int &idx) {
    if (idx + 4 >= MAX_NMEA_DATA_LENGTH) return ZP_ERROR_MEMORY_OVERFLOW;
    int32_t val;
    memcpy(&val, &processBuffer[idx], 4);
    tempData.vx = val / 100.0f;
    return ZP_ERROR_OK;
}

ZP_ERROR_e GPS::getVy(int &idx) {
    if (idx + 4 >= MAX_NMEA_DATA_LENGTH) return ZP_ERROR_MEMORY_OVERFLOW;
    int32_t val;
    memcpy(&val, &processBuffer[idx], 4);
    tempData.vy = val / 100.0f;
    return ZP_ERROR_OK;
}

ZP_ERROR_e GPS::getVz(int &idx) {
    if (idx + 4 >= MAX_NMEA_DATA_LENGTH) return ZP_ERROR_MEMORY_OVERFLOW;
    int32_t val;
    memcpy(&val, &processBuffer[idx], 4);
    tempData.vz = val / 100.0f;
    return ZP_ERROR_OK;
}