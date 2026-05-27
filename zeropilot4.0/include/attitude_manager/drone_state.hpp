#pragma once

typedef struct {
    float pitch;      // Pitch angle in radians
    float roll;       // Roll angle in radians
    float yaw;        // Yaw angle in radians
    float altitude;   // Altitude in meters
    float airspeed;   // Airspeed in m/s
    float rollRate;   // Roll rate in deg/s
    float pitchRate;  // Pitch rate in deg/s
    float yawRate;    // Yaw rate in deg/s
} DroneState_t;

// Default drone state initialization
static const DroneState_t DRONE_STATE_DEFAULT = {
    0.0f,   // pitch
    0.0f,   // roll
    0.0f,   // yaw
    0.0f,   // altitude
    0.0f,   // airspeed
    0.0f,   // roll rate
    0.0f,   // pitch rate
    0.0f    // yaw rate
};
