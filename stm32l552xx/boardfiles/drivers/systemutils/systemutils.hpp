#pragma once

#include "systemutils_iface.hpp"
#include "error.h"

class SystemUtils : public ISystemUtils {
    public:
        SystemUtils() = default;
        ZP_ERROR_e delayMs(uint32_t delay_ms) override;
        ZP_ERROR_e getCurrentTimestampMs(uint32_t *timestamp_ms) override;
};
