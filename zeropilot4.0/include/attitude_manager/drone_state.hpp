#pragma once

typedef struct {
    float pitch;      // Pitch angle in radians
    float roll;       // Roll angle in radians
    float yaw;        // Yaw angle in radians
    float altitude;   // Altitude in meters
    float airspeed;   // Airspeed in m/s
    float rollRate;   // Roll rate in rad/s
    float pitchRate;  // Pitch rate in rad/s
    float yawRate;    // Yaw rate in rad/s
    float verticalVel; // Vertical velocity in m/s
    float verticalAcc; // Vertical acceleration in m/s^2
} DroneState_t;

// Default drone state initialization
static const DroneState_t DRONE_STATE_DEFAULT = {
    0.0f,   // Pitch
    0.0f,   // Roll
    0.0f,   // Yaw
    0.0f,   // Altitude
    0.0f,   // Airspeed
    0.0f,   // Roll rate
    0.0f,   // Pitch rate
    0.0f,   // Yaw rate
    0.0f,   // Vertical velocity
    0.0f    // Vertical acceleration
};
