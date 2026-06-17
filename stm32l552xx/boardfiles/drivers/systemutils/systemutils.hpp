#pragma once

#include "systemutils_iface.hpp"
#include <stdint.h>

class SystemUtils : public ISystemUtils {
    public:
        SystemUtils() = default;
        ZP_ERROR_e delayMs(uint32_t delay_ms) override;
        ZP_ERROR_e getCurrentTimestampMs(uint32_t& currentTime) override;

        ZP_ERROR_e profilerGetAll(TaskProfile* out, uint8_t* count) override;
        ZP_ERROR_e profilerRegister(const char* name, uint8_t* outId) override;
        ZP_ERROR_e profilerBegin(uint8_t id) override;
        ZP_ERROR_e profilerEnd(uint8_t id) override;

    private:
        ZP_ERROR_e dwtInit();
};
