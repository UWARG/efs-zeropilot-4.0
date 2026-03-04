#pragma once

#include <cmath>
#include <cstdint>
#include "stm32l5xx.h"
#include "gps_iface.hpp"

typedef enum {
    GPS_HALF_CPLT_CALLBACK,
    GPS_CPLT_CALLBACK,
    GPS_IDLE_DETECTED_CALLBACK
} GpsCallbackStatus_e;

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
    void processGPSData(GpsCallbackStatus_e status);

private:
    GpsData_t validData;
    GpsData_t tempData;

    uint8_t rxBuffer[MAX_NMEA_DATA_LENGTH + RX_BUFFER_PADDING_SIZE];
    uint8_t* processBuffer = rxBuffer;
    UART_HandleTypeDef *huart;

    HAL_StatusTypeDef enableMessage(uint8_t msgClass, uint8_t msgId);
    bool sendUBX(uint8_t *msg, uint16_t len);
    void calcChecksum(uint8_t *msg, uint16_t len);

    bool parseRMC();
    bool parseGGA();
    bool parseUBX();

    // UBX helper functions
    bool getVx(int &idx);
    bool getVy(int &idx);
    bool getVz(int &idx);

    // RMC helper functions
    bool getTimeRMC(int &idx);
    bool getLatitudeRMC(int &idx);
    bool getLongitudeRMC(int &idx);
    bool getSpeedRMC(int &idx);
    bool getTrackAngleRMC(int &idx);
    bool getDateRMC(int &idx);

    // GGA helper functions
    bool getNumSatellitesGGA(int &idx);
    bool getAltitudeGGA(int &idx);
};
