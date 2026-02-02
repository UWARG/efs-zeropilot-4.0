#pragma once

typedef struct {
    float pitch;      // Pitch angle in degrees
    float roll;       // Roll angle in degrees
    float yaw;        // Yaw angle in degrees
    float altitude;   // Altitude in meters
    float airspeed;   // Airspeed in m/s
} DroneState_t;

// Default drone state initialization
static const DroneState_t DRONE_STATE_DEFAULT = {
    0.0f,   // pitch
    0.0f,   // roll
    0.0f,   // yaw
    0.0f,   // altitude
    0.0f    // airspeed
};
