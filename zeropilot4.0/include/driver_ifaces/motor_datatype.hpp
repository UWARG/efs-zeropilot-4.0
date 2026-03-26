#pragma once

#include "motor_iface.hpp"

typedef struct {
    IMotorControl *motorInstance; 
    bool isInverted;
    int trim; // trim value to adjust motor output, can be positive or negative in range [-50,50]
} MotorInstance_t;

typedef struct {   
    MotorInstance_t *motors;
    uint8_t motorCount;
} MotorGroupInstance_t;
