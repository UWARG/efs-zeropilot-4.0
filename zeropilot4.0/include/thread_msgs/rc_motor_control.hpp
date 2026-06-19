#pragma once
#include <cstdint>
#include "zp_error.h"

// Flight modes for PLANE: numbering aligns to ArduPilot's MAVLink mapping for MissionPlanner compatibility
enum class PlaneFlightMode_e : uint32_t {
    MANUAL  = 0,
    FBWA    = 5
};

inline ZP_ERROR_e isValidPlaneFlightMode(uint32_t val) {
    switch (static_cast<PlaneFlightMode_e>(val)) {
        case PlaneFlightMode_e::MANUAL:
        case PlaneFlightMode_e::FBWA:
            return ZP_ERROR_OK;
        default:
            return ZP_ERROR_INVALID_PARAM;
    }
}

typedef struct {
    float roll;
    float pitch;
    float yaw;
    float throttle;
    bool arm;
    float flapAngle;
    PlaneFlightMode_e flightMode;
} RCMotorControlMessage_t;
