#pragma once 

#include "stm32l5xx.h"
#include "barometer_iface.hpp"
#include <cmath>
#include <cstdint>

static constexpr uint8_t ICP20100_POWER_MODE = (1U << 2);
static constexpr uint8_t ICP20100_FORCED_MES_TRIGGER = (1U << 4);
static constexpr uint8_t ICP20100_TRIGGER_COMMAND_MEAS = (ICP20100_POWER_MODE | ICP20100_FORCED_MES_TRIGGER);

class Barometer : public IBarometer {

    public:
        Barometer(I2C_HandleTypeDef *hi2c);
        bool readData(BaroData_t &data);
        bool init(); 
        void rxCallback();
        bool firWarmupPoll();
        void computeAltitude(BaroData_t *data);
        I2C_HandleTypeDef* getI2C();
       
    private:
        enum State_e {
            NOT_STARTED,
            FIFO_STARTED,
            DATA_READ
        };
        I2C_HandleTypeDef *hi2c;
        volatile bool dataFilled = 0;
        volatile State_e callbackState = NOT_STARTED;
        volatile bool initiatedRead = false;
        uint8_t pressTempData[6];
        uint8_t fifoRegister;
        float latestTemperatureC = 0.0f;
        float latestPressureKpa = 0.0f;
        bool readRegister(uint16_t memAddress, uint8_t * pData, uint16_t size);
		bool writeRegister(uint16_t memAddress, uint8_t * pData, uint16_t size);
};