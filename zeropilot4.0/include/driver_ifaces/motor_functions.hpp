#pragma once

#include <cstdint>

enum class MotorFunction_e : int16_t {
    GPIO            = -1,
    Disabled        = 0,
    Flap            = 2,
    Aileron         = 4,
    Elevator        = 19,
    Rudder          = 21,
    GroundSteering  = 26,
    Throttle        = 70,
};
