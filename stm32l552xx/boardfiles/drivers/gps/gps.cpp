#include <cstring>

#include "gps.hpp"

GPS::GPS(UART_HandleTypeDef* huart) : huart(huart) {}

UART_HandleTypeDef* GPS::getHUART() {
    return huart;
}

UART_HandleTypeDef* GPS::getHUART() {
    return huart;
}

ZP_ERROR_e GPS::init() {
    HAL_StatusTypeDef success = HAL_UARTEx_ReceiveToIdle_DMA(
		huart,
		rxBuffer,
		MAX_NMEA_DATA_LENGTH
    );

    if (success != HAL_OK) {
        return ZP_ERROR_FAIL;
    }

    __HAL_UART_ENABLE_IT(huart, UART_IT_IDLE);

    ZP_ERROR_e messagesuccess = enableMessage(0x01, 0x11); //enable ubx velecef messages

    if (messagesuccess != ZP_ERROR_OK) {
        return messagesuccess;
    }

    return ZP_ERROR_OK;
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

ZP_ERROR_e GPS::readData(GpsData_t *data) {
    if (data == nullptr) {
        return ZP_ERROR_NULLPTR;
    }

    __HAL_DMA_DISABLE_IT(huart->hdmarx, DMA_IT_TC);

    *data = validData;
    validData.isNew = false;

    __HAL_DMA_ENABLE_IT(huart->hdmarx, DMA_IT_TC);

    return ZP_ERROR_OK;
}

ZP_ERROR_e GPS::processGPSData() {
    __HAL_DMA_DISABLE(huart->hdmarx);

    ZP_ERROR_e result = parseRMC();
    if (result != ZP_ERROR_OK) {
        __HAL_DMA_ENABLE(huart->hdmarx);
        return result;
    }

    result = parseGGA();
    if (result != ZP_ERROR_OK) {
        __HAL_DMA_ENABLE(huart->hdmarx);
        return result;
    }

    result = parseUBX();
    if (result != ZP_ERROR_OK) {
        __HAL_DMA_ENABLE(huart->hdmarx);
        return result;
    }

    tempData.isNew = true;
    validData = tempData;

    validData.isNew = false;

   __HAL_DMA_ENABLE_IT(huart->hdmarx, DMA_IT_TC);

    return ZP_ERROR_OK;
}
void GPS::processGPSData() {
    memcpy(processBuffer, rxBuffer, MAX_NMEA_DATA_LENGTH);
//
ZP_ERROR_e GPS::parseUBX() {
    int idx = 0;

    // find sync
    while (!(processBuffer[idx] == 0xB5 && processBuffer[idx+1] == 0x62)) {
        idx++;
        if (idx >= MAX_NMEA_DATA_LENGTH) return ZP_ERROR_PARSE;  // not found
    }
    // check class
    if (rxBuffer[idx+2] != 0x01 || rxBuffer[idx+3] != 0x11) {
        return ZP_ERROR_PARSE;
    }

    // payload
    idx += 6;

    idx += 4;

    ZP_ERROR_e result = getVx(idx);
    if (result != ZP_ERROR_OK) {
    	return result;
    }

    idx += 4;

    result = getVy(idx);
    if (result != ZP_ERROR_OK) {
    	return result;
    }

    idx += 4;

    result = getVz(idx);
    if (result != ZP_ERROR_OK) {
    	return result;
    }

    return ZP_ERROR_OK;
}


ZP_ERROR_e GPS::parseRMC() {
    int idx = 0;
    while (!(processBuffer[idx] == 'R' && processBuffer[idx+1] == 'M' && processBuffer[idx+2] == 'C')) {
        idx++;
        if (idx == MAX_NMEA_DATA_LENGTH) return ZP_ERROR_PARSE;
    }

    idx += 4;

    // Check if data exists
    if (rxBuffer[idx] == ',') {
        return ZP_ERROR_PARSE;
    }

    ZP_ERROR_e result = getTimeRMC(idx);
    if (result != ZP_ERROR_OK) {
        return result;
    }

    // Skip to status
    while (processBuffer[idx] != ',') idx++;

    // Begin status
    idx++;

    // Check if data valid
    if (rxBuffer[idx] == 'V') return ZP_ERROR_PARSE;
    // End status

    idx += 2;

    result = getLatitudeRMC(idx);
    if (result != ZP_ERROR_OK) {
        return result;
    }

    idx += 2;

    result = getLongitudeRMC(idx);
    if (result != ZP_ERROR_OK) {
        return result;
    }

    idx += 2;

    result = getSpeedRMC(idx);
    if (result != ZP_ERROR_OK) {
        return result;
    }

    while (processBuffer[idx] != ',') idx++;
    idx++;

    result = getTrackAngleRMC(idx);
    if (result != ZP_ERROR_OK) {
        return result;
    }

    while (processBuffer[idx] != ',') idx++;
    while (processBuffer[idx] == ',') idx++;

    result = getDateRMC(idx);
    if (result != ZP_ERROR_OK) {
        return result;
    }

    return ZP_ERROR_OK;
}

ZP_ERROR_e GPS::parseGGA() {
    int idx = 0;
    while (!(processBuffer[idx] == 'G' && processBuffer[idx + 1] == 'G' && processBuffer[idx + 2] == 'A')) {
        idx++;
        if (idx == MAX_NMEA_DATA_LENGTH) return ZP_ERROR_PARSE;
    }
    idx+=4; // Skip to data

    // Check if data exists
    if (rxBuffer[idx] == ',') {
        return ZP_ERROR_PARSE;
    }

    // Skip 7 sections of data
    for (int i = 0; i < 6; i++, idx++) {
        while (processBuffer[idx] != ',') idx++;
    }

    ZP_ERROR_e result = getNumSatellitesGGA(idx);
    if (result != ZP_ERROR_OK) {
        return result;
    }

    for (int i = 0; i < 2; i++, idx++) {
    	while (processBuffer[idx] != ',') idx++;
    }

    result = getAltitudeGGA(idx);
    if (result != ZP_ERROR_OK) {
    	return result;
    }

    return ZP_ERROR_OK;
}

ZP_ERROR_e GPS::getTimeRMC(int &idx) {
    uint8_t hour = (rxBuffer[idx] - '0') * 10 + (rxBuffer[idx + 1] - '0');
    idx += 2;
    uint8_t minute = (processBuffer[idx] - '0') * 10 + (processBuffer[idx + 1] - '0');
    idx += 2;
    uint8_t second = (processBuffer[idx] - '0') * 10 + (processBuffer[idx + 1] - '0');

    tempData.time.hour = hour;
    tempData.time.minute = minute;
    tempData.time.second = second;

    return ZP_ERROR_OK;
}

ZP_ERROR_e GPS::getLatitudeRMC(int &idx) {
    float lat = 0;
    for (int i = 0; i < 2; i++, idx++) {
        lat *= 10;
        lat += ((float)(processBuffer[idx] - '0'));
    }

    float lat_minutes = 0;
    while (processBuffer[idx] != '.') {
        lat_minutes *= 10;
        lat_minutes += ((float)(processBuffer[idx] - '0'));
        idx++;
    }
    idx++; // Skip decimal char

    // Including two digits of minutes
    uint32_t mult = 10;
    while (processBuffer[idx] != ',' && mult <= DECIMAL_PRECISION) {
        lat_minutes += ((float)(processBuffer[idx] - '0')) / mult;
        idx++;
        mult *= 10;
    }

    lat += lat_minutes/60;

    tempData.latitude = lat;

    // Skip to NS char indicator
    while (processBuffer[idx] != ',') idx++;
    idx++; // Skip over comma
    tempData.latitude *= (processBuffer[idx] == 'N') ? 1 : -1;

    return ZP_ERROR_OK;
}

ZP_ERROR_e GPS::getLongitudeRMC(int &idx) {
    float lon = 0;
    for (int i = 0; i < 3; i++, idx++) {
        lon *= 10;
        lon += ((float)(processBuffer[idx] - '0'));
    }

    float lon_minutes = 0;
    while (processBuffer[idx] != '.') {
        lon_minutes *= 10;
        lon_minutes += ((float)(processBuffer[idx] - '0'));
        idx++;
    }

    idx++; // Skip decimal char

    // Including two digits of minutes
    uint32_t mult = 10;
    while (processBuffer[idx] != ',' && mult <= DECIMAL_PRECISION) {
        lon_minutes += ((float)(processBuffer[idx] - '0')) / mult;
        idx++;
        mult *= 10;
    }

    lon += lon_minutes / 60;

    tempData.longitude = lon;
    while (processBuffer[idx] != ',') idx++;
    idx++;
    tempData.longitude *= (processBuffer[idx] == 'E') ? 1 : -1;

    return ZP_ERROR_OK;
}

ZP_ERROR_e GPS::getSpeedRMC(int &idx) {
    float spd = 0;
    while (processBuffer[idx] != '.') {
        spd *= 10;
        spd += processBuffer[idx] - '0';
        idx++;
    }
    idx++; // Decimal char
    uint32_t mult = 10;
    while (processBuffer[idx] != ',' && mult <= DECIMAL_PRECISION) {
        spd += ((float)(processBuffer[idx] - '0')) / mult;
        idx++;
        mult *= 10;
    }

    tempData.groundSpeed = spd * 51.4444; // Convert from kt to cm/s

    return ZP_ERROR_OK;
}

ZP_ERROR_e GPS::getTrackAngleRMC(int &idx) {
    float cog = 0;
    // Check if cog was calculated
    if (processBuffer[idx] != ',') {
        while (processBuffer[idx] != '.') {
            cog *= 10;
            cog += processBuffer[idx] - '0';
            idx++;
        }
        idx++; // Decimal char
        uint32_t mult = 10;
        while (processBuffer[idx] != ',' && mult <= DECIMAL_PRECISION) {
            cog += ((float)(processBuffer[idx] - '0')) / mult;
            idx++;
            mult *= 10;
        }
    }
    else {
        cog = INVALID_TRACK_ANGLE;
    }

    tempData.trackAngle = cog;

    return ZP_ERROR_OK;
}

ZP_ERROR_e GPS::getDateRMC(int &idx) {
    int day = (rxBuffer[idx] - '0') * 10 + rxBuffer[idx + 1] - '0';
    idx += 2;
    int month = (processBuffer[idx] - '0') * 10 + processBuffer[idx + 1] - '0';
    idx += 2;
    int year = (processBuffer[idx] - '0') * 10 + processBuffer[idx + 1] - '0';

    tempData.time.day = day;
    tempData.time.month= month;
    tempData.time.year = year;

    return ZP_ERROR_OK;
}

ZP_ERROR_e GPS::getNumSatellitesGGA(int &idx) {
    int numSats = 0;
    while (processBuffer[idx] != ',') {
        numSats *= 10;
        numSats += processBuffer[idx] - '0';
        idx++;
    }

    tempData.numSatellites = numSats;

    return ZP_ERROR_OK;
}

ZP_ERROR_e GPS::getAltitudeGGA(int &idx) {
    float altitude = 0;
    while (processBuffer[idx] != '.') {
        altitude *= 10;
        altitude += processBuffer[idx] - '0';
        idx++;
    }
    idx++; // Decimal char
    uint32_t mult = 10;
    while (processBuffer[idx] != ',' && mult <= DECIMAL_PRECISION) {
        altitude += ((float)(processBuffer[idx] - '0')) / mult;
        idx++;
        mult *= 10;
    }

    tempData.altitude = altitude;
    return ZP_ERROR_OK;
}


ZP_ERROR_e GPS::getVx(int &idx) {
    int32_t ecefVX = (int32_t)((uint32_t)rxBuffer[idx] |
                               ((uint32_t)rxBuffer[idx+1] << 8) |
                               ((uint32_t)rxBuffer[idx+2] << 16) |
                               ((uint32_t)rxBuffer[idx+3] << 24));

    tempData.vx = ecefVX / 100.0f;
    return ZP_ERROR_OK;
}

ZP_ERROR_e GPS::getVy(int &idx) {
    int32_t ecefVY = (int32_t)((uint32_t)rxBuffer[idx] |
                               ((uint32_t)rxBuffer[idx+1] << 8) |
                               ((uint32_t)rxBuffer[idx+2] << 16) |
                               ((uint32_t)rxBuffer[idx+3] << 24));

    tempData.vy = ecefVY / 100.0f;
    return ZP_ERROR_OK;
}

ZP_ERROR_e GPS::getVz(int &idx) {
    int32_t ecefVZ = (int32_t)((uint32_t)rxBuffer[idx] |
                               ((uint32_t)rxBuffer[idx+1] << 8) |
                               ((uint32_t)rxBuffer[idx+2] << 16) |
                               ((uint32_t)rxBuffer[idx+3] << 24));

    tempData.vz = ecefVZ / 100.0f;
    return ZP_ERROR_OK;
}
