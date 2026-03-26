#pragma once

#include "stm32l5xx.h"
#include "gps_iface.hpp"
#include "gps_datatypes.hpp"
#include "gps_defines.hpp"
#include "error.h"
#include <cmath>

class GPS : public IGPS {

public:
    ZP_ERROR_e readData(GpsData_t *data) override;

    GPS(UART_HandleTypeDef *huart);

    ZP_ERROR_e init();
    ZP_ERROR_e processGPSData();

private:
    GpsData_t validData;
    GpsData_t tempData;

    uint8_t rxBuffer[MAX_NMEA_DATA_LENGTH];
    uint8_t processBuffer[MAX_NMEA_DATA_LENGTH];
    UART_HandleTypeDef *huart;

    ZP_ERROR_e enableMessage(uint8_t msgClass, uint8_t msgId);
    ZP_ERROR_e sendUBX(uint8_t *msg, uint16_t len);
    ZP_ERROR_e calcChecksum(uint8_t *msg, uint16_t len);

    ZP_ERROR_e parseRMC();
    ZP_ERROR_e parseGGA();
    ZP_ERROR_e parseUBX();

    // UBX helper functions
    ZP_ERROR_e getVx(int &idx);
    ZP_ERROR_e getVy(int &idx);
    ZP_ERROR_e getVz(int &idx);


    // RMC helper functions
    ZP_ERROR_e getTimeRMC(int &idx);
    ZP_ERROR_e getLatitudeRMC(int &idx);
    ZP_ERROR_e getLongitudeRMC(int &idx);
    ZP_ERROR_e getSpeedRMC(int &idx);
    ZP_ERROR_e getTrackAngleRMC(int &idx);
    ZP_ERROR_e getDateRMC(int &idx);



    // GGA helper functions
    ZP_ERROR_e getNumSatellitesGGA(int &idx);
    ZP_ERROR_e getAltitudeGGA(int &idx);
};
