#pragma once

#include <cstdint>
#include "zp_error.h"

class IStatusLed {
    protected:
        IStatusLed() = default;

    public:
        virtual ~IStatusLed() = default;

        // Set the LED to a solid color, each channel is 0-255
        virtual ZP_Error setColor(uint8_t red, uint8_t green, uint8_t blue) = 0;
};
