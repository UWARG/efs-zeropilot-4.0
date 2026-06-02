#pragma once

#include "systemutils_iface.hpp"
#include <stdint.h>

class SystemUtils : public ISystemUtils {
    public:
        SystemUtils() = default;
        void delayMs(uint32_t delay_ms) override;
        uint32_t getCurrentTimestampMs() override;
        void profilerGetAll(TaskProfile* out, uint8_t* count) override;

        static void dwtInit();
        static void profilerRegister(const char* name, uint8_t* outId);
        static void profilerBegin(uint8_t id);
        static void profilerEnd(uint8_t id);
};






