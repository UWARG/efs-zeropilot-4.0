#pragma once

#include "rc_motor_control.hpp"
#include "drone_state.hpp"
#include <cstdint>

class Flightmode {
    protected:
        Flightmode() = default;

        #ifdef FIXED_WING
        static constexpr uint8_t NUM_MOTORS = 0;
        #endif
        #ifdef QUADCOPTER
        static constexpr uint8_t NUM_MOTORS = 4;
        #endif

    public:
        virtual ~Flightmode() = default;

        virtual void activateFlightMode() = 0;
        virtual RCMotorControlMessage_t runControl(RCMotorControlMessage_t controlInput, const DroneState_t &droneState) = 0;

        virtual const float *getMixedMotors() = 0;
};
