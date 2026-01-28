#pragma once

#include "motor_iface.hpp"

typedef struct {
    IMotorControl *motorInstance; 
    bool isInverted;
} MotorInstance_t;

typedef struct {   
    MotorInstance_t *motors;
    uint8_t motorCount;
} MotorGroupInstance_t;
