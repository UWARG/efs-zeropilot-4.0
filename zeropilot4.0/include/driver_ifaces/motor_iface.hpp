#pragma once

#include <cstdint>

class IMotorControl {
    protected: 
        IMotorControl() = default;
        
    public:
        virtual ~IMotorControl() = default;

        // set pwm percentage of servo motors
        virtual void set(uint32_t percent) = 0;

        // get servo index
        virtual uint8_t getServoIdx() const = 0;
};
