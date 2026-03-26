#pragma once

#include <cstdint>
#include "error.h"

class IMotorControl {
    protected:
        IMotorControl() = default;

    public:
        virtual ~IMotorControl() = default;

        // set pwm percentage of servo motors
        virtual ZP_ERROR_e set(uint32_t percent) = 0;

        // get servo index
        virtual ZP_ERROR_e getServoIdx(uint8_t *idx) const = 0;
};
