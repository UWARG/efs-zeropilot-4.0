#pragma once

#include "motor_iface.hpp"
#include "motor_functions.hpp"

typedef struct {
    IMotorControl *motorInstance; 
    bool isInverted;
    int trim;   // neutral position for motor
    int min;    // min position for motor
    int max;    // max position for motor
    MotorFunction_e function;
} MotorInstance_t;

typedef struct {   
    MotorInstance_t *motors;
    uint8_t motorCount;
} MotorGroupInstance_t;
