#pragma once

#include "stm32l5xx_hal.h"
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
        GPS(UART_HandleTypeDef *huart);

        UART_HandleTypeDef* getHuart();

        GpsProtocol_t getProtocol();

        ZP_Error readData(GpsData_t &data) override;

        ZP_Error init();
        void rxCallback(uint16_t size);
        ZP_Error restartDMA();

    private:
        GpsProtocol_t protocol = NMEA;
        GpsData_t tempData{};

        volatile uint8_t rxBuffer[MAX_NMEA_DATA_LENGTH] = {0};
        volatile uint8_t processBuffer[MAX_NMEA_DATA_LENGTH] = {0};
        volatile uint8_t *processBufferEnd = (uint8_t*)processBuffer;
        volatile bool parsingData = false;
        volatile bool dataReady = false;
        UART_HandleTypeDef *huart;

        ZP_Error configureUBX();
        ZP_Error setMessageRate(uint8_t msgClass, uint8_t msgId, uint8_t rate);
        ZP_Error setRate(uint16_t measRateMs, uint16_t navRate);
        ZP_Error configValset(uint32_t key, uint32_t value);
        ZP_Error waitForAck(uint8_t msgClass, uint8_t msgId);
        ZP_Error receiveByte(uint8_t &byte, uint32_t deadline);
        ZP_Error sendUBX(uint8_t *msg, uint16_t len);
        void calcChecksum(uint8_t *msg, uint16_t len);

        uint16_t processBufferLen();
        ZP_Error incrementProcessBufferIndex(uint16_t &idx, uint16_t increment);

        // Both advance idx past the frame they consumed, so readData() always makes progress even when the frame is corrupted 
        ZP_Error consumeUBX(uint16_t &idx);
        ZP_Error consumeNMEA(uint16_t &idx);

        bool verifyChecksumUBX(uint16_t start, uint16_t frameLen);
        bool verifyChecksumNMEA(uint16_t start, uint16_t end);
        bool matchesSentenceType(uint16_t idx, const char *sentenceType);

        ZP_Error parseRMC(uint16_t &idx);
        ZP_Error parseGGA(uint16_t &idx);
        ZP_Error parseVELECEF(uint16_t &idx);
        ZP_Error parsePVT(uint16_t &idx);

        // UBX helper functions
        uint16_t getLenUBX(uint16_t &idx);

        // RMC helper functions
        ZP_Error getTimeRMC(uint16_t &idx);
        ZP_Error getLatitudeRMC(uint16_t &idx);
        ZP_Error getLongitudeRMC(uint16_t &idx);
        ZP_Error getSpeedRMC(uint16_t &idx);
        ZP_Error getTrackAngleRMC(uint16_t &idx);
        ZP_Error getDateRMC(uint16_t &idx);

        // GGA helper functions
        ZP_Error getNumSatellitesGGA(uint16_t &idx);
        ZP_Error getAltitudeGGA(uint16_t &idx);

        // VELECEF helper functions
        ZP_Error getVxVELECEF(uint16_t &idx);
        ZP_Error getVyVELECEF(uint16_t &idx);
        ZP_Error getVzVELECEF(uint16_t &idx);
};
