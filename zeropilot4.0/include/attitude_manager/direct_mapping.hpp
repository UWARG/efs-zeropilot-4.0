#pragma once

#include "flightmode.hpp"
#include "error.h"

class DirectMapping : public Flightmode {
    public:
        DirectMapping() = default;

        ZP_ERROR_e runControl(RCMotorControlMessage_t *motorOutputs, RCMotorControlMessage_t controlInput) override;
};
