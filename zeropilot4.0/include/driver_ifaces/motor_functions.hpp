#pragma once

#include <cstdint>

enum class MotorFunction_e : int16_t {
    GPIO            = -1,
    DISABLED        = 0,
    FLAP            = 2,
    AILERON         = 4,
    ELEVATOR        = 19,
    RUDDER          = 21,
    GROUND_STEERING = 26,
    THROTTLE        = 70,
};
