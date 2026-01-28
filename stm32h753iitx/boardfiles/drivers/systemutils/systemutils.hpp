#pragma once

#include "systemutils_iface.hpp"

class SystemUtils : public ISystemUtils {
    public:
        SystemUtils() = default;
        void delayMs(uint32_t delay_ms) override;
        uint32_t getCurrentTimestampMs() override;
};
