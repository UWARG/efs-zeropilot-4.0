#pragma once

#include "rc_motor_control.hpp"
#include "drone_state.hpp"

class Flightmode {
    protected:
        Flightmode() = default;

    public:
        virtual ~Flightmode() = default;

        virtual RCMotorControlMessage_t runControl(RCMotorControlMessage_t controlInput, const DroneState_t &droneState) = 0;
};
