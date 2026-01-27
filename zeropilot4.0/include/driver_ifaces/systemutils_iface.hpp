#pragma once

#include <cstdint>
#include "error.h"

class ISystemUtils {
    protected:
        ISystemUtils() = default;

    public:
        virtual ~ISystemUtils() = default;

        virtual ZP_ERROR_e delayMs(uint32_t delay_ms) = 0;
        virtual ZP_ERROR_e getCurrentTimestampMs(uint32_t *timestamp_ms) = 0;
};
