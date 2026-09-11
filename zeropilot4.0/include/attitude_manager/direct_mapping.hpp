#pragma once

#include "flightmode.hpp"
#include "zp_error.h"

class DirectMapping : public Flightmode {
    public:
        DirectMapping() = default;

        ZP_Error activateFlightMode() override;
        ZP_Error runControl(RCMotorControlMessage_t &motorOutputs, RCMotorControlMessage_t controlInput, const DroneState_t &droneState) override;
};