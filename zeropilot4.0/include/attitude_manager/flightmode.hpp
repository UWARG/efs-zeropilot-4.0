#pragma once

#include "rc_motor_control.hpp"
#include "drone_state.hpp"
#include <cstdint>

class Flightmode {
    protected:
        Flightmode() = default;

    public:
        virtual ~Flightmode() = default;

        virtual void activateFlightMode() = 0;
        virtual RCMotorControlMessage_t runControl(RCMotorControlMessage_t controlInput, const DroneState_t &droneState) = 0;

        // Whether the active flight mode needs GPS to arm/operate.
        virtual bool requiresGPS() const { return false; }
};
