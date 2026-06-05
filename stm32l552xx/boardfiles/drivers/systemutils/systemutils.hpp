#pragma once

#include "systemutils_iface.hpp"
#include <stdint.h>

class SystemUtils : public ISystemUtils {
    public:
        SystemUtils() = default;
        void delayMs(uint32_t delay_ms) override;
        uint32_t getCurrentTimestampMs() override;
        void profilerGetAll(TaskProfile* out, uint8_t* count) override;

        void profilerRegister(const char* name, uint8_t* outId);
        void profilerBegin(uint8_t id);
        void profilerEnd(uint8_t id);
        
    private:
        void dwtInit();
};






