#pragma once

#include <cstdint>

class ISystemUtils {
    protected:
        ISystemUtils() = default;

    public:
        virtual ~ISystemUtils() = default;
        
        virtual void delayMs(uint32_t delay_ms) = 0;
        virtual uint32_t getCurrentTimestampMs() = 0;
};
