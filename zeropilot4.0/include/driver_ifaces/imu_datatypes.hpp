#pragma once

#include <cstdint>

// unscaled, NED frame
typedef struct {
    int16_t xacc;
    int16_t yacc;
    int16_t zacc;
    int16_t xgyro;
    int16_t ygyro;
    int16_t zgyro;
} RawImu_t;

typedef struct {
    float xacc;
    float yacc;
    float zacc;
    float xgyro;
    float ygyro;
    float zgyro;
} ScaledImu_t;

// Attitude in radians
typedef struct {
    float roll;
    float pitch;
    float yaw;
} Attitude_t;