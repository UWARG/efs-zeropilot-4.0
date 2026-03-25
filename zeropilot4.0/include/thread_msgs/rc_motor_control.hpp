#pragma once
#include <cstdint>

// Flight modes for PLANE: numbering aligns to ArduPilot's MAVLink mapping for MissionPlanner compatibility
enum class PlaneFlightMode_e : uint32_t {
    MANUAL  = 0,
    FBWA    = 5,
    FBWB    = 6
};

inline bool isValidPlaneFlightMode(uint32_t val) {
    switch (static_cast<PlaneFlightMode_e>(val)) {
        case PlaneFlightMode_e::MANUAL:
        case PlaneFlightMode_e::FBWA:
            return true;
        default:
            return false;
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
