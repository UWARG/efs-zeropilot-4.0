#pragma once

#include "flightmode.hpp"
#include "zp_error.h"

class DirectMapping : public Flightmode {
    public:
        DirectMapping() = default;

        ZP_ERROR_e activateFlightMode() override;
        ZP_ERROR_e runControl(RCMotorControlMessage_t &motorOutputs, RCMotorControlMessage_t controlInput, const DroneState_t &droneState) override;
};