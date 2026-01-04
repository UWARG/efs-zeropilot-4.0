#pragma once

#include "stm32l5xx_hal.h"
#include "power_module_iface.hpp"
#include "power_module_defines.hpp"
#include <cmath> 


class PowerModule : public IPowerModule {
    public:
        bool readData(PMData_t *data) override;
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
