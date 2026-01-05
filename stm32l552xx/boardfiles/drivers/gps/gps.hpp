#pragma once

#include "stm32l5xx.h"
#include "gps_iface.hpp"
#include "gps_datatypes.hpp"
#include "gps_defines.hpp"
#include <cmath>

class GPS : public IGPS {

public:
    UART_HandleTypeDef* getHUART();

    GpsData_t readData() override;

    GPS(UART_HandleTypeDef *huart);

    bool init();
    void processGPSData();

private:
    GpsData_t validData;
    GpsData_t tempData;

    uint8_t rxBuffer[MAX_NMEA_DATA_LENGTH];
    uint8_t processBuffer[MAX_NMEA_DATA_LENGTH];
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
