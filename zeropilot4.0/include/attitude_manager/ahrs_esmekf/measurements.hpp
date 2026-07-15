#pragma once

#include "arm_math.h"

#include <cstdint>

class Measurements {
public:
    static constexpr uint32_t VECTOR_SIZE = 3;

    Measurements(
        const float32_t *gyroInitial = nullptr,
        const float32_t *accelInitial = nullptr,
        const float32_t *magInitial = nullptr
    );

    void updateGyro(const float32_t *gyroNewIn);
    void updateAccel(const float32_t *accelNewIn);
    void updateMag(const float32_t *magNewIn);

    void updateBiases(
        const float32_t *gyroBiasNew,
        const float32_t *accelBiasNew,
        const float32_t *magBiasNew
    );

    void updateGyroBar();
    void updateAccelBar();
    void updateMagBar();
    void updateAllBars();

    float32_t gyroPrev[VECTOR_SIZE];
    float32_t gyroNew[VECTOR_SIZE];
    float32_t gyroBar[VECTOR_SIZE];

    float32_t accelPrev[VECTOR_SIZE];
    float32_t accelNew[VECTOR_SIZE];
    float32_t accelBar[VECTOR_SIZE];

    float32_t magPrev[VECTOR_SIZE];
    float32_t magNew[VECTOR_SIZE];
    float32_t magBar[VECTOR_SIZE];

    float32_t gyroBiasAccumulated[VECTOR_SIZE];
    float32_t accelBiasAccumulated[VECTOR_SIZE];
    float32_t magBiasAccumulated[VECTOR_SIZE];

private:
    void copyVector3(const float32_t *vIn, float32_t *vOut);
    void zeroVector3(float32_t *vOut);
    void averageVector3(const float32_t *a, const float32_t *b, float32_t *out);
};
