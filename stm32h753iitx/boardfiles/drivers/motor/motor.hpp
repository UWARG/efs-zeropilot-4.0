#pragma once

#include "motor_iface.hpp"
#include "stm32h7xx_hal.h"
#include "zp_error.h"

class MotorControl : public IMotorControl {
    public:
        MotorControl(TIM_HandleTypeDef *timer, uint32_t timerChannel, uint32_t minDutyCycle, uint32_t maxDutyCycle, uint8_t servoIdx);
        
        /**
         * @brief sets PWM motor output
         * @param percent PWM value 0-100
         */
        ZP_ERROR_e set(uint32_t percent) override;

        /**
         * @brief starts PWM output
         */
        ZP_ERROR_e init() override;

        /**
         * @brief enables servo output
         * @param 
         */
        static void enableServo(GPIO_TypeDef* enGpioBase, uint16_t enGpioNum);

        /**
         * @brief enables servo switch
         * @param 
         */
        static ZP_ERROR_e enableServoSwitch(GPIO_TypeDef* csGpioBase, uint16_t csGpioNum, SPI_HandleTypeDef *hspi);

    private:
        TIM_HandleTypeDef * const timer;
        const uint32_t timerChannel;
        const uint32_t minCCR;
        const uint32_t maxCCR;

        const uint8_t servoIdx;
};
