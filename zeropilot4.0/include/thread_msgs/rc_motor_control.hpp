#pragma once
#include <cstdint>

// Flight modes for PLANE: numbering aligns to ArduPilot's MAVLink mapping for MissionPlanner compatibility
#define PLANE_FLIGHT_MODES \
    X(MANUAL, 0) \
    X(FBWA, 5)

enum class PlaneFlightMode_e : uint32_t {
    #define X(name, val) name = val,
    PLANE_FLIGHT_MODES
    #undef X
};

inline bool isValidPlaneFlightMode(uint32_t val) {
    switch (static_cast<PlaneFlightMode_e>(val)) {
        #define X(name, val) case PlaneFlightMode_e::name:
        PLANE_FLIGHT_MODES
        #undef X
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
