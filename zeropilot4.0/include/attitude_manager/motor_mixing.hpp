#pragma once
#include "rc_motor_control.hpp"
#include "motor_datatype.hpp"
#include <cmath>

class MotorMixing{
    public:
        static float *quadMotorMixer(const RCMotorControlMessage_t OUTPUT_CONTROL_MSG, MotorGroupInstance_t *mainMotorGroup, float* motorPercent);

        static float *fixedWingMoterMixer(const RCMotorControlMessage_t OUTPUT_CONTROL_MSG,  MotorGroupInstance_t *mainMotorGroup, float* motorPercent);
};
