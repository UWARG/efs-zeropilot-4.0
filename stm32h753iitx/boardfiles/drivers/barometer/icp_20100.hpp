#pragma once 

#include "stm32h7xx.h"
#include "barometer_iface.hpp"
#include <cmath>
#include <cstdint>

// Register Definitions for Mikroe ICP-20100

static constexpr uint16_t ICP20100_I2C_ADDR = (0x64U << 1); // Shift by 1 for HAL
static constexpr uint8_t ICP20100_REG_MODE_SELECT = 0xC0U;
static constexpr uint8_t ICP20100_DEVICE_ID = 0x0CU;
static constexpr uint8_t ICP20100_MASTER_LOCK = 0xBEU;
static constexpr uint8_t ICP20100_OTP_CONFIG_1 = 0xACU;
static constexpr uint8_t ICP20100_OTP_STATUS = 0xB9U;
static constexpr uint8_t ICP20100_OTP_STATUS2 = 0xBFU;
static constexpr uint8_t ICP20100_VERSION_REG = 0xD3U;
static constexpr uint8_t ICP20100_OTP_DBG2 = 0xBCU;
static constexpr uint8_t ICP20100_OTP_MRA_LSB = 0xAFU;
static constexpr uint8_t ICP20100_OTP_MRA_MSB = 0xB0U;
static constexpr uint8_t ICP20100_OTP_MRB_LSB = 0xB1U;
static constexpr uint8_t ICP20100_OTP_MRB_MSB = 0xB2U;
static constexpr uint8_t ICP20100_OTP_MR_LSB = 0xADU;
static constexpr uint8_t ICP20100_OTP_MR_MSB = 0xAEU;
static constexpr uint8_t ICP20100_OTP_ADDRESS = 0xB5U;
static constexpr uint8_t ICP20100_OTP_COMMAND = 0xB6U;
static constexpr uint8_t ICP20100_OTP_RDATA = 0xB8U;
static constexpr uint8_t ICP20100_TRIM1_MSB = 0x05U;
static constexpr uint8_t ICP20100_TRIM2_LSB = 0x06U;
static constexpr uint8_t ICP20100_TRIM2_MSB = 0x07U;
static constexpr uint8_t ICP20100_FIFO_CONFIG = 0xC3U;
static constexpr uint8_t ICP20100_INTERRUPT_MASK = 0xC2U;
static constexpr uint8_t ICP20100_REG_MODE_SELECT_KEY = 0x04U;
static constexpr uint8_t ICP20100_MASTER_UNLOCK_KEY = 0x1FU;
static constexpr uint8_t ICP20100_MASTER_LOCK_KEY = 0x00U;
static constexpr uint8_t ICP20100_OTP_ENABLE_BOTH = 0x03U;
static constexpr uint8_t ICP20100_OTP_STATUS2_BOOTUP = 0x01U;
static constexpr uint8_t ICP20100_PRESS_DATA_0 = 0xFAU;
static constexpr uint8_t ICP20100_FIFO_FILL = 0xC4U;
static constexpr uint8_t ICP20100_DEVICE_STATUS = 0xCDU;
static constexpr uint8_t ICP20100_MODE_SYNC_STATUS_BIT = 0x01U;


static constexpr uint8_t ICP20100_POWER_MODE = (1U << 2);
static constexpr uint8_t ICP20100_FORCED_MES_TRIGGER = (1U << 4);
static constexpr uint8_t ICP20100_TRIGGER_COMMAND_MEAS = (ICP20100_POWER_MODE | ICP20100_FORCED_MES_TRIGGER);

class Barometer : public IBarometer {
    public:
        Barometer(I2C_HandleTypeDef *hi2c);
        bool readData(BaroData_t *data);
        bool init(); 
        void rxCallback();
        bool firWarmupPoll();
        void computeAltitude(BaroData_t *data);
       
    private:
        I2C_HandleTypeDef *hi2c;
        volatile bool dataFilled = 0;
		volatile uint8_t callbackCount = 0; 
        volatile bool initiatedRead = false;
        uint8_t Press_Temp_Data[6];
        uint8_t FIFO_REGISTER;
        float latestTemperatureC = 0.0f;
        float latestPressurekPa = 0.0f;
        bool readRegister(uint16_t memAddress, uint8_t * pData, uint16_t size, I2C_HandleTypeDef *hi2c);
		bool writeRegister(uint16_t memAddress, uint8_t * pData, uint16_t size, I2C_HandleTypeDef *hi2c);
};