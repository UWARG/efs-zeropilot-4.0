#pragma once

#include "arm_math.h"

#include <cstdint>

void multiplyQuaternions(const float32_t *q1, const float32_t *q2, float32_t *qOut);

void inverseQuaternion(const float32_t *qIn, float32_t *qOut);

void normalizeQuaternion(const float32_t *qIn, float32_t *qOut);

void averageQuaternions(const float32_t *q1, const float32_t *q2, float32_t *qOut);

void bToIFrameRotMatrix(const float32_t *qIn, float32_t *cOut);

void iToBFrameRotMatrix(const float32_t *qIn, float32_t *cOut);

void quaternionExponential(const float32_t *rotationVector, float32_t *qOut);

void rotateVector(const float32_t *vIn, const float32_t *qIn, float32_t *vOut);

float32_t angularDistanceDegrees(const float32_t *qTrue, const float32_t *qEst);

void quatToEuler(const float32_t *qIn, float32_t *eulerOut);
