#pragma once

#include <cstdint>
#include "zp_error.h"

class IMotorControl {
    protected: 
        IMotorControl() = default;

        bool armFlag = false;
        
    public:
        virtual ~IMotorControl() = default;

        // Set pwm percentage of servo motors
        virtual ZP_Error set(uint32_t percent) = 0;

        // Initialize/start motor output
        virtual ZP_Error init() = 0;

        // Set arm flag
        virtual void setArm(bool arm) { armFlag = arm; };
};
