#pragma once

#include "stm32h7xx_hal.h"
#include "rangefinder_iface.hpp"

class Rangefinder : public IRangefinder {
    public:
        Rangefinder(I2C_HandleTypeDef *hi2c);

        int init() override;
        RangefinderData_t readData() override;

        void txCallback();
        void rxCallback();
        void errorCallback();

        I2C_HandleTypeDef *getI2C();

    private:
        I2C_HandleTypeDef *hi2c;
        RangefinderData_t data;
        volatile bool dataFilled = false;

        static constexpr uint8_t READ_RESPONSE_LENGTH = 9;
        uint8_t rxBuffer[READ_RESPONSE_LENGTH] = {0};

        void restartDma();
        uint8_t computeChecksum();

};