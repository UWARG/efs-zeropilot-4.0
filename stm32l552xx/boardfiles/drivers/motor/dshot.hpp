#pragma once

#include "motor_iface.hpp"
#include "stm32l5xx_hal.h"

static constexpr uint8_t DSHOT_FRAME_LEN = 16;  // 11 bits throttle + 1 bit telem + 4 bits CRC
static constexpr uint8_t DSHOT_BUF_LEN = DSHOT_FRAME_LEN + 1;   // Extra idle slot to hold line low after frame

class DshotMotorControl : public IMotorControl{
    public:
        DshotMotorControl(TIM_HandleTypeDef *timer, uint32_t timerChannel, bool telReq);

        /**
         * @brief sets dshot throttle output
         * @param percent throttle percentage(0-100), 0 sends disarm command
         */
        void set(uint32_t percent) override;

        /**
         * @brief starts arming sequence for ESC
         */
        void init();

    private: 
        TIM_HandleTypeDef * const timer;
        const uint32_t timerChannel;
        const uint8_t telReq;

        uint16_t updateBuffer[DSHOT_BUF_LEN] = {0};
        uint16_t dmaBuffer[DSHOT_BUF_LEN] = {0};
};
