#pragma once

#include "rc_motor_control.hpp"
#include "drone_state.hpp"
#include <cstdint>

class Flightmode {
    protected:
        Flightmode() = default;

    public:
        virtual ~Flightmode() = default;

        virtual RCMotorControlMessage_t runControl(RCMotorControlMessage_t controlInput, const DroneState_t &droneState) = 0;
};

// Flight modes for PLANE: numbering aligns to ArduPilot's MAVLink mapping for MissionPlanner compatibility
enum class PlaneFlightMode_e : uint32_t {
    MANUAL  = 0,
    FBWA    = 5
};
