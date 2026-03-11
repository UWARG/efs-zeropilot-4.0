#pragma once
#include <cstdint>

// Flight modes for PLANE: numbering aligns to ArduPilot's MAVLink mapping for MissionPlanner compatibility
enum class PlaneFlightMode_e : uint32_t {
    MANUAL  = 0,
    FBWA    = 5,
    FBWB    = 6
};

typedef struct {
    float roll;
    float pitch;
    float yaw;
    float throttle;
    bool arm;
    float flapAngle;
    PlaneFlightMode_e flightMode;
} RCMotorControlMessage_t;
