#pragma once

#include "magnetometer_iface.hpp"
#include "zp_error.h"
#include "stm32h7xx_hal.h"
#include <cstdint>

class Magnetometer : public IMagnetometer {
    public:
        // csPort may be nullptr if CS is strapped high
        Magnetometer(I2C_HandleTypeDef *hi2c,
                     GPIO_TypeDef *csPort = nullptr,
                     uint16_t csPin = 0);

        // For a board that straps A0/A1 to a non default I2C address
        Magnetometer(I2C_HandleTypeDef *hi2c,
                     GPIO_TypeDef *csPort,
                     uint16_t csPin,
                     uint8_t address);

        ZP_Error init() override;
        ZP_Error readData(MagData_t &data) override;

        // Called from HAL I2C callbacks in override.cpp
        void masterTxCpltCallback();
        void masterRxCpltCallback();
        void errorCallback();

        I2C_HandleTypeDef *getI2C();

    private:
        // Sizes rxBuffer, so it has to stay in the header
        static constexpr uint8_t MEASUREMENT_BYTES = 7;

        // Type of the state member, so it has to stay in the header
        typedef enum {
            IDLE,
            SM_TX,
            SM_RX,
            WAIT_CONV,
            RM_TX,
            RM_RX,
        } ReadState_e;

        I2C_HandleTypeDef *hi2c;
        GPIO_TypeDef *csPort;
        uint16_t csPin;
        const uint16_t halAddress;

        uint8_t gain;
        uint8_t xRes;
        uint8_t yRes;
        uint8_t zRes;
        uint8_t hallconf;
        uint8_t tcmpEn;
        uint8_t filter;
        uint8_t osr;

        uint8_t status;
        uint16_t regValue;

        bool initialized;
        volatile ReadState_e state;
        volatile bool sampleReady;
        // Raw ZP_Error bits raised from the I2C callbacks, drained by readData()
        volatile uint32_t pendingError;
        uint32_t conversionDoneMs;
        uint32_t transferStartMs;

        uint8_t txByte;
        uint8_t rxBuffer[MEASUREMENT_BYTES];

        MagData_t magData;

        ZP_Error transceive(uint8_t *txData, uint16_t txSize, uint8_t *rxData, uint16_t rxSize);
        ZP_Error sendCommand(uint8_t command);
        ZP_Error writeRegister(uint8_t reg, uint16_t value);
        ZP_Error readRegister(uint8_t reg);
        ZP_Error setGain(uint8_t newGain);
        ZP_Error setResolution(uint8_t newXRes, uint8_t newYRes, uint8_t newZRes);
        ZP_Error setFilter(uint8_t newFilter);
        ZP_Error setOversampling(uint8_t newOsr);
        ZP_Error refreshConfigCache();

        ZP_Error startMeasurement();
        ZP_Error readMeasurement();
        ZP_Error decode();
        uint32_t conversionTimeMs() const;
};
