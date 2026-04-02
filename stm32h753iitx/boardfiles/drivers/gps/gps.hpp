#pragma once

#include "stm32h7xx.h"
#include "gps_iface.hpp"
#include <cmath>

static constexpr uint8_t MAX_NMEA_DATA_LENGTH_PER_LINE = 82;
static constexpr uint8_t NUM_NMEA_DATA_LINES = 8;
static constexpr uint16_t MAX_NMEA_DATA_LENGTH = MAX_NMEA_DATA_LENGTH_PER_LINE * NUM_NMEA_DATA_LINES;
static constexpr uint32_t DECIMAL_PRECISION = 1e6;
static constexpr uint16_t RX_BUFFER_PADDING_SIZE = 16;
static constexpr uint16_t RX_BUFFER_SIZE = 2 * MAX_NMEA_DATA_LENGTH;

class GPS : public IGPS {

public:
    UART_HandleTypeDef* getHUART();

    GpsData_t readData() override;

    GPS(UART_HandleTypeDef *huart);

    bool init();
    void rxCallback(uint16_t size);

private:
    GpsData_t validData;
    GpsData_t tempData;

    uint8_t rxBuffer[MAX_NMEA_DATA_LENGTH];
    uint8_t processBuffer[MAX_NMEA_DATA_LENGTH];
    uint8_t *processBufferEnd = processBuffer;
    UART_HandleTypeDef *huart;

    HAL_StatusTypeDef enableMessage(uint8_t msgClass, uint8_t msgId);
    bool sendUBX(uint8_t *msg, uint16_t len);
    void calcChecksum(uint8_t *msg, uint16_t len);
    bool incrementProcessBufferIndex(uint16_t &idx, uint16_t increment);

    bool parseRMC();
    bool parseGGA();
    bool parseUBX();

    // UBX helper functions
    bool getVx(uint16_t &idx);
    bool getVy(uint16_t &idx);
    bool getVz(uint16_t &idx);

    // RMC helper functions
    bool getTimeRMC(uint16_t &idx);
    bool getLatitudeRMC(uint16_t &idx);
    bool getLongitudeRMC(uint16_t &idx);
    bool getSpeedRMC(uint16_t &idx);
    bool getTrackAngleRMC(uint16_t &idx);
    bool getDateRMC(uint16_t &idx);

    // GGA helper functions
    bool getNumSatellitesGGA(uint16_t &idx);
    bool getAltitudeGGA(uint16_t &idx);
};
