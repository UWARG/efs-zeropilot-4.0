#pragma once 

#include "stm32l5xx_hal.h"
#include "barometer_iface.hpp"
#include <cmath>
#include <cstdint>

struct RegInfoBarometer {
    uint8_t address;
    uint8_t byte_size;
};

// Datasheet Values

static constexpr uint8_t VERSION_2 = 0xB2;
static constexpr uint8_t BOOT_FINISHED = 1U;

// Bit Shift Values
static constexpr uint8_t kBootStatusBitPos = 0U;
static constexpr uint8_t kBootStatusEnabledValue = 1U;

class Barometer : public IBarometer {
    public:
        Barometer(I2C_HandleTypeDef *hi2c);
        bool readData(BaroData_t &data);
        bool init(); 
        void rxCallback();
        bool firWarmupPoll();
        void computeAltitude(BaroData_t *data);
        I2C_HandleTypeDef* getI2C(); 
        void waitForOTPRead();
    private:
        I2C_HandleTypeDef *hi2c;
        volatile bool dataFilled = 0;
		volatile uint8_t callbackCount; 
        volatile bool initiatedRead = false;
        uint8_t Press_Temp_Data[6];
        uint8_t FIFO_REGISTER;
        float latestTemperatureC = 0.0f;
        float latestPressurekPa = 0.0f;
        bool readRegister(uint16_t memAddress, uint8_t * pData, uint16_t size);
		bool writeRegister(uint16_t memAddress, uint8_t * pData, uint16_t size);
};