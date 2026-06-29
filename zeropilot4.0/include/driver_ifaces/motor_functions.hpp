#pragma once

#include <cstdint>

enum class MotorFunction_e : int16_t {
    GPIO            = -1,
    DISABLED        = 0,
    #ifdef FIXED_WING
    FLAP            = 2,
    AILERON         = 4,
    ELEVATOR        = 19,
    RUDDER          = 21,
    GROUND_STEERING = 26,
    THROTTLE        = 70,
    #endif
    #ifdef QUADCOPTER
    MOTOR_1 = 33, // Front right
    MOTOR_2 = 34, // Rear left 
    MOTOR_3 = 35, // Front left
    MOTOR_4 = 36  // Rear right
    #endif
};
