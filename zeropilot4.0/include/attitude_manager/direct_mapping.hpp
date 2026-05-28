#pragma once

#include "flightmode.hpp"

class DirectMapping : public Flightmode {
    public:
        DirectMapping() = default;

        void activateFlightMode() override;
        RCMotorControlMessage_t runControl(RCMotorControlMessage_t controlInput, const DroneState_t &droneState) override;

        const float *getMixedMotors() override;
    
    private:
        float motor_percent[NUM_MOTORS] = {0};
};
