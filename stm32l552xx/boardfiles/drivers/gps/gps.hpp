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

        UART_HandleTypeDef* getHUART();

        GpsProtocol_t getProtocol();

        GpsData_t readData() override;

        bool init();
        void rxCallback(uint16_t size);
        void restartDMA();

    private:
        GpsProtocol_t protocol = NMEA;
        GpsData_t tempData{};

        volatile uint8_t rxBuffer[MAX_NMEA_DATA_LENGTH] = {0};
        volatile uint8_t processBuffer[MAX_NMEA_DATA_LENGTH] = {0};
        volatile uint8_t *processBufferEnd = (uint8_t*)processBuffer;
        volatile bool parsingData = false;
        volatile bool dataReady = false;
        UART_HandleTypeDef *huart;

        bool configureUBX();
        bool setMessageRate(uint8_t msgClass, uint8_t msgId, uint8_t rate);
        bool setMessageRateValset(uint32_t key, uint8_t rate);
        bool waitForAck(uint8_t msgClass, uint8_t msgId);
        bool receiveByte(uint8_t &byte, uint32_t deadline);
        bool sendUBX(uint8_t *msg, uint16_t len);
        void calcChecksum(uint8_t *msg, uint16_t len);

        uint16_t processBufferLen();
        bool incrementProcessBufferIndex(uint16_t &idx, uint16_t increment);

        // Both advance idx past the frame they consumed, so readData() always makes progress even when the frame is corrupted 
        bool consumeUBX(uint16_t &idx);
        bool consumeNMEA(uint16_t &idx);

        bool verifyChecksumUBX(uint16_t start, uint16_t frameLen);
        bool verifyChecksumNMEA(uint16_t start, uint16_t end);
        bool matchesSentenceType(uint16_t idx, const char *sentenceType);

        bool parseRMC(uint16_t &idx);
        bool parseGGA(uint16_t &idx);
        bool parseVELECEF(uint16_t &idx);
        bool parsePVT(uint16_t &idx);

        // UBX helper functions
        uint16_t getLenUBX(uint16_t &idx);

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

        // VELECEF helper functions
        bool getVxVELECEF(uint16_t &idx);
        bool getVyVELECEF(uint16_t &idx);
        bool getVzVELECEF(uint16_t &idx);
};
