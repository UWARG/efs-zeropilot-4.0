#pragma once
#include <cstdint>

// Flight modes: numbering aligns to ArduPilot's MAVLink mapping for MissionPlanner compatibility
enum class FlightMode_e : uint32_t {
    #ifdef FIXED_WING
    MANUAL  = 0,
    FBWA    = 5
    #endif
    #ifdef QUADCOPTER
    ACRO  = 1   // verify later
    #endif
};

inline bool isValidFlightMode(uint32_t val) {
    switch (static_cast<FlightMode_e>(val)) {
        #ifdef FIXED_WING
        case FlightMode_e::MANUAL:
        case FlightMode_e::FBWA:
            return true;
        default:
            return false;
        #endif

        #ifdef QUADCOPTER
        case FlightMode_e::ACRO:
            return true;
        default:
            return false;
        #endif
    }
}

typedef struct {
    float roll;
    float pitch;
    float yaw;
    float throttle;
    bool arm;
    #ifdef FIXED_WING
    float flapAngle;
    #endif
    FlightMode_e flightMode;
} RCMotorControlMessage_t;
