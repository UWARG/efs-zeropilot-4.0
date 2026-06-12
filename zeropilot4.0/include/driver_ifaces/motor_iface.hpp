#pragma once

#include <cstdint>

class IMotorControl {
    protected: 
        IMotorControl() = default;

        bool armFlag = false;
        
    public:
        virtual ~IMotorControl() = default;

        // Set pwm percentage of servo motors
        virtual void set(uint32_t percent) = 0;

        // Initialize/start motor output
        virtual void init() = 0;

        // Set arm flag
        virtual void setArm(bool arm) { armFlag = arm; };
};
