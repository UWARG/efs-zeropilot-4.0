#pragma once
#include <cstdint>
#include "zp_error.h"

// Flight modes: numbering aligns to ArduPilot's MAVLink mapping for MissionPlanner compatibility
enum class FlightMode_e : uint32_t {
    #ifdef PLANE
    MANUAL  = 0,
    FBWA    = 5
    #endif
    #ifdef QUADCOPTER
    STABILIZE = 0,
    ACRO = 1
    #endif
};

inline ZP_Error isValidFlightMode(uint32_t val) {
    switch (static_cast<FlightMode_e>(val)) {
        #ifdef PLANE
        case FlightMode_e::MANUAL:
        case FlightMode_e::FBWA:
        #endif
        #ifdef QUADCOPTER
        case FlightMode_e::ACRO:
        case FlightMode_e::STABILIZE:
        #endif
            return ZP_ERROR_OK;
        default:
            return ZP_ERROR_INVALID_ARG;
    }
}

typedef struct {
    float roll;
    float pitch;
    float yaw;
    float throttle;
    bool arm;
    #ifdef PLANE
    float flapAngle;
    #endif
    FlightMode_e flightMode;
} RCMotorControlMessage_t;
