#pragma once

#include "systemutils_iface.hpp"
#include "stm32h7xx_hal.h"
#include <stdint.h>

class SystemUtils : public ISystemUtils {
    public:
        SystemUtils() = default;
        void delayMs(uint32_t delay_ms) override;
        uint32_t getCurrentTimestampMs() override;

        void profilerGetAll(TaskProfile* out, uint8_t* count) override;
        void profilerRegister(const char* name, uint8_t* outId) override;
        void profilerBegin(uint8_t id) override;
        void profilerEnd(uint8_t id) override;

        static void dwtInit() {
            if (DWT->CYCCNT & DWT_CTRL_CYCCNTENA_Msk) return; // DWT is already running
            CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
            DWT->CYCCNT = 0;
            DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
        }
};
