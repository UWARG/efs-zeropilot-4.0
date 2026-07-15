#pragma once

#include "arm_math.h"
#include <cstdint>

extern const float32_t IDENTITY_QUATERNION[4];
extern const float32_t GRAVITY_INERTIAL[3];
extern const float32_t MAGNETOMETER_INERTIAL[3];

bool normalizeVector(const float32_t *vIn, float32_t *vOut, uint32_t length);

void skewSymmetric(const float32_t *vIn, float32_t *sOut);

void ensureSymmetricMatrix(const float32_t *aIn, float32_t *aOut, uint32_t rows, uint32_t cols);

void copyVector(const float32_t *vIn, float32_t *vOut, uint32_t length);

void bToIFrameRotMatrix(const float32_t *qIn, float32_t *cOut);
