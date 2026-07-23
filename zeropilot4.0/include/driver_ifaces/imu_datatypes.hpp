#pragma once

#include <cstdint>

// Unscaled, FRD frame
typedef struct {
    int16_t xacc;
    int16_t yacc;
    int16_t zacc;
    int16_t xgyro;
    int16_t ygyro;
    int16_t zgyro;
    uint32_t timestamp;
    uint8_t imuId;
} RawImu_t;

typedef struct {
    float xacc; // m/s^2
    float yacc; // m/s^2
    float zacc; // m/s^2
    float xgyro; // deg/s
    float ygyro; // deg/s
    float zgyro; // deg/s
    uint32_t timestamp;
    uint8_t imuId;
} ScaledImu_t;

// Attitude in radians
typedef struct {
    float roll;
    float pitch;
    float yaw;
} Attitude_t;

// Gyro Bias in radians
typedef struct {
    float x;
    float y;
    float z;
} GyroBias_t;

typedef struct {
    RawImu_t *data;
    uint16_t count;
    uint32_t readTime; // Time the fifo(last data packet) was read, in us
} RawImuBatch_t;

typedef struct {
    ScaledImu_t *data;
    uint16_t count;
    uint32_t readTime; // Time the fifo(last data packet) was read, in us
} ScaledImuBatch_t;

// deg/s
typedef struct {
    float x;
    float y;
    float z;
} GyroStartupBias_t;
