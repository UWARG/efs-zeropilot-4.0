#pragma once

#include "systemutils_iface.hpp"
#include "stm32l5xx_hal.h"
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

        static void dwtInit();
        static uint32_t getDWTMicroSec();

        float cmsis_dsp_sinf(float x) override;
        float cmsis_dsp_cosf(float x) override;
};
