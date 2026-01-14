#pragma once

#include "stm32l5xx_hal.h"
#include "power_module_iface.hpp"
#include <cmath>

#pragma once

#include <cstdint>


struct RegInfo {
    uint8_t address;
    uint8_t byte_size;
    bool is_signed;
};
// Register definitions for INA228
static constexpr RegInfo REG_CONFIG = {0x00, 2, false}; // 16
static constexpr RegInfo REG_ADC_CONFIG = {0x01, 2, false}; // 16
static constexpr RegInfo REG_SHUNT_CAL = {0x02, 2, false}; // 16
static constexpr RegInfo REG_VBUS = {0x05, 3, false}; // 24
static constexpr RegInfo REG_CURRENT = {0x07, 3, true};  // 24
static constexpr RegInfo REG_POWER = {0x08, 3, false}; // 24
static constexpr RegInfo REG_ENERGY = {0x09, 5, false}; // 40
static constexpr RegInfo REG_CHARGE = {0x0A, 5, true};  // 40

static constexpr uint8_t INA228_ADDR = 0b1000101; // VS to VS

//physical constants
static constexpr float RSHUNT = 0.0005; 
static constexpr float IMAX = 32.0f;  //I could make this 16A as well
static constexpr float CURRENT_LSB = IMAX / (1 << 19);  
static constexpr uint16_t SHUNT_CAL_VALUE = (13107.2e6 * CURRENT_LSB * RSHUNT);  
static constexpr float VBUS_LSB = 195.3125e-6f;  // 195.3125 uV per bit
static constexpr float VSHUNT_LSB = 312.5e-9f;    // 312.5 nV per bit
static constexpr float POWER_LSB = 3.2f * CURRENT_LSB;  
static constexpr float ENERGY_LSB = 16 * 3.2 * CURRENT_LSB;  
static constexpr float CHARGE_LSB = CURRENT_LSB;  

// Config values
static constexpr uint16_t CONFIG_VALUE = 0x8000;  
static constexpr uint16_t ADC_CONFIG_VALUE = 0xF002; // 0b 1111 000 000 000 010, continuous reading and 16 samples averaged

static constexpr uint8_t REGISTERS_TO_READ = 5;



class PowerModule : public IPowerModule {
    public:
        bool readData(PMData_t *data);
        PowerModule(I2C_HandleTypeDef *hi2c);
        bool init();
        volatile uint8_t callbackCount;
        void I2C_MemRxCpltCallback();
        I2C_HandleTypeDef* getI2C();


    private:
        PMData_t processedData;
        I2C_HandleTypeDef *hi2c;
        bool writeRegister(uint16_t MemAddress, uint8_t * pData, uint16_t Size, I2C_HandleTypeDef *hi2c);
        bool readRegister(uint16_t MemAddress, uint8_t * pData, uint16_t Size, I2C_HandleTypeDef *hi2c);
        void parse(I2C_HandleTypeDef *hi2c);


        uint8_t vbusData[3];
        uint8_t currentData[3];
        uint8_t powerData[3];
        uint8_t energyData[5];
        uint8_t chargeData[5];
        volatile bool dataFilled = 0;
};
