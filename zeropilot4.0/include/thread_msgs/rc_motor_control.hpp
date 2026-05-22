#pragma once
#include <cstdint>

// Flight modes for PLANE: numbering aligns to ArduPilot's MAVLink mapping for MissionPlanner compatibility
enum class PlaneFlightMode_e : uint32_t {
    MANUAL  = 0,
    FBWA    = 5
};

// Flight modes for COPTER: numbering aligns to ArduPilot's MAVLink mapping for MissionPlanner compatibility
enum class CopterFlightMode_e : uint32_t {
    ACRO  = 1   // verify later
};

inline bool isValidPlaneFlightMode(uint32_t val) {
    #ifdef FIXED_WING
    switch (static_cast<PlaneFlightMode_e>(val)) {
        case PlaneFlightMode_e::MANUAL:
        case PlaneFlightMode_e::FBWA:
            return true;
        default:
            return false;
    }
    #endif

    #ifdef QUADCOPTER
    switch (static_cast<CopterFlightMode_e>(val)) {
        case CopterFlightMode_e::ACRO:
            return true;
        default:
            return false;
    }
    #endif

}

typedef struct {
    float roll;
    float pitch;
    float yaw;
    float throttle;
    bool arm;

    #ifdef FIXED_WING
    float flapAngle;
    PlaneFlightMode_e flightMode;
    #endif

    #ifdef QUADCOPTER
    CopterFlightMode_e flightMode;
    #endif
} RCMotorControlMessage_t;
