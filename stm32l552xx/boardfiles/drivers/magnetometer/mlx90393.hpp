#pragma once

#include "magnetometer_iface.hpp"
#include "stm32l5xx_hal.h"
#include <cstdint>

enum : uint8_t {
    MLX90393_AXIS_T = 0x01,
    MLX90393_AXIS_X = 0x02,
    MLX90393_AXIS_Y = 0x04,
    MLX90393_AXIS_Z = 0x08,
};

enum : uint8_t {
    MLX90393_CONF1 = 0x00,
    MLX90393_CONF2 = 0x01,
    MLX90393_CONF3 = 0x02,
    MLX90393_CONF4 = 0x03,
};

enum : uint8_t {
    MLX90393_CMD_NOP = 0x00,
    MLX90393_CMD_EXIT = 0x80,
    MLX90393_CMD_START_BURST = 0x10,
    MLX90393_CMD_WAKE_ON_CHANGE = 0x20,
    MLX90393_CMD_START_MEASUREMENT = 0x30,
    MLX90393_CMD_READ_MEASUREMENT = 0x40,
    MLX90393_CMD_READ_REGISTER = 0x50,
    MLX90393_CMD_WRITE_REGISTER = 0x60,
    MLX90393_CMD_MEMORY_RECALL = 0xD0,
    MLX90393_CMD_MEMORY_STORE = 0xE0,
    MLX90393_CMD_RESET = 0xF0,
};

enum : uint8_t {
    MLX90393_STATUS_BURST_MODE = 0x80,
    MLX90393_STATUS_WAKE_ON_CHANGE = 0x40,
    MLX90393_STATUS_POLLING_MODE = 0x20,
    MLX90393_STATUS_ERROR = 0x10,
    MLX90393_STATUS_EEC = 0x08,
    MLX90393_STATUS_RESET = 0x04,
};

enum : uint8_t {
    MLX90393_GAIN_5X = 0x00,
    MLX90393_GAIN_4X = 0x01,
    MLX90393_GAIN_3X = 0x02,
    MLX90393_GAIN_2_5X = 0x03,
    MLX90393_GAIN_2X = 0x04,
    MLX90393_GAIN_1_67X = 0x05,
    MLX90393_GAIN_1_33X = 0x06,
    MLX90393_GAIN_1X = 0x07,
};

enum : uint8_t {
    MLX90393_RES_15 = 0x00,
    MLX90393_RES_16 = 0x01,
    MLX90393_RES_17 = 0x02,
    MLX90393_RES_18 = 0x03,
};

// 7 bit; datasheet family is 0x0C..0x0F
static constexpr uint8_t MLX90393_DEFAULT_ADDR = 0x18;

class Magnetometer : public IMagnetometer {
    public:
        // csPort may be nullptr if CS is strapped high
        Magnetometer(I2C_HandleTypeDef *hi2c,
                     GPIO_TypeDef *csPort = nullptr,
                     uint16_t csPin = 0,
                     uint8_t address = MLX90393_DEFAULT_ADDR);

        bool init() override;
        MagData_t readData() override;

        void startCalibration() override;
        void cancelCalibration() override;
        MagCalStatus_t getCalibrationStatus() override;
        const MagCalConstants_t &getCalibrationConstants() const override;
        void resetCalibrationToDefaults();

        // Called from HAL I2C callbacks in override.cpp
        void masterTxCpltCallback();
        void masterRxCpltCallback();
        void errorCallback();

        I2C_HandleTypeDef *getI2C();

        static constexpr uint32_t CAL_COLLECT_DURATION_MS = 30000;

    private:
        static constexpr uint8_t ZYXT = MLX90393_AXIS_X | MLX90393_AXIS_Y | MLX90393_AXIS_Z;
        static constexpr uint8_t MEASUREMENT_BYTES = 7;
        static constexpr uint32_t CONVERSION_MARGIN_MS = 10;
        static constexpr uint32_t TRANSFER_TIMEOUT_MS = 100;
        static constexpr uint32_t INIT_TIMEOUT_MS = 100;

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

        volatile ReadState_e state;
        volatile bool sampleReady;
        uint32_t conversionDoneMs;
        uint32_t transferStartMs;

        uint8_t txByte;
        uint8_t rxBuffer[MEASUREMENT_BYTES];

        MagData_t magData;

        bool transceive(uint8_t *txData, uint16_t txSize, uint8_t *rxData, uint16_t rxSize);
        bool sendCommand(uint8_t command);
        bool writeRegister(uint8_t reg, uint16_t value);
        bool readRegister(uint8_t reg);
        bool setGain(uint8_t newGain);
        bool setResolution(uint8_t newXRes, uint8_t newYRes, uint8_t newZRes);
        bool setFilter(uint8_t newFilter);
        bool setOversampling(uint8_t newOsr);
        bool refreshConfigCache();

        void startMeasurement();
        void readMeasurement();
        void decode();
        uint32_t conversionTimeMs() const;

        static constexpr uint32_t CAL_MIN_SAMPLES = 60;
        static constexpr float CAL_MIN_AXIS_SPAN_UT = 10.0f;
        static constexpr float CAL_MIN_SPAN_RATIO = 0.5f;
        static constexpr float CAL_MIN_FIELD_UT = 5.0f;
        static constexpr float CAL_MAX_FIELD_UT = 200.0f;

        MagCalConstants_t calConstants;
        MagCalState_t calState;
        MagCalError_t calError;
        uint32_t calStartMs;
        uint32_t calSampleCount;

        float calNormalMatrix[16];
        float calNormalVector[4];
        float calAxisMin[3];
        float calAxisMax[3];

        void calResetAccumulators();
        void calAddSample(const float field[3], uint32_t nowMs);
        void calApply(float field[3]) const;
        bool calSolve();
        static bool invert4x4(const float src[16], float dst[16]);
};
