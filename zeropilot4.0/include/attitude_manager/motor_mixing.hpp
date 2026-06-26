#pragma once
#include "rc_motor_control.hpp"
#include "motor_datatype.hpp"
#include <cmath>

class MotorMixing{
    public:
        #ifdef FIXED_WING
        static void fixedWingMoterMixer(const RCMotorControlMessage_t OUTPUT_CONTROL_MSG,  MotorGroupInstance_t *mainMotorGroup, float* motorPercent);
        #endif
        #ifdef QUADCOPTER
        static void quadMotorMixer(const RCMotorControlMessage_t OUTPUT_CONTROL_MSG, MotorGroupInstance_t *mainMotorGroup, float* motorPercent);
        #endif
};
