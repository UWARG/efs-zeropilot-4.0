#pragma once

#include "rc_motor_control.hpp"
#include "drone_state.hpp"
#include <cstdint>
#include "zp_error.h"

class Flightmode {
    protected:
        Flightmode() = default;

    public:
        virtual ~Flightmode() = default;

        virtual ZP_ERROR_e activateFlightMode() = 0;
        virtual ZP_ERROR_e runControl(RCMotorControlMessage_t &controlOutput, RCMotorControlMessage_t controlInput, const DroneState_t &droneState) = 0;
};
