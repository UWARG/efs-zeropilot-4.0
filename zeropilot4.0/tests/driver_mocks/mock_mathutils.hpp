#pragma once

#include "mathutils_iface.hpp"
#include <gmock/gmock.h>

class MockMathUtils : public IMathUtils {
    public:
        // --- Vector Math ---
        MOCK_METHOD(float, vectorNorm, (const float* src, uint16_t dim), (override));
        MOCK_METHOD(bool, vectorNormalize, (const float* src, float* dst, uint16_t dim), (override));

        // --- Matrix Operations ---
        MOCK_METHOD(bool, matrixAdd, (const float* srcA, const float* srcB, float* dst, uint16_t rows, uint16_t cols), (override));
        MOCK_METHOD(bool, matrixSub, (const float* srcA, const float* srcB, float* dst, uint16_t rows, uint16_t cols), (override));
        MOCK_METHOD(bool, matrixMult, (const float* srcA, uint16_t rowsA, uint16_t colsA, const float* srcB, uint16_t colsB, float* dst), (override));
        MOCK_METHOD(bool, matrixTranspose, (const float* src, uint16_t rows, uint16_t cols, float* dst), (override));
        MOCK_METHOD(bool, matrixScale, (const float* src, float scale, float* dst, uint16_t rows, uint16_t cols), (override));
        MOCK_METHOD(bool, matrixInverse, (const float* src, uint16_t dim, float* dst), (override));
        MOCK_METHOD(void, skewSymmetric, (const float* v3, float* dst3x3), (override));
        MOCK_METHOD(bool, ensureSymmetric, (float* M, uint16_t dim), (override));

        // --- Quaternion Operations (q = [w, x, y, z]) ---
        MOCK_METHOD(void, quatMultiply, (const float* q1, const float* q2, float* qOut), (override));
        MOCK_METHOD(void, quatInverse, (const float* q, float* qOut), (override));
        MOCK_METHOD(void, quatNormalize, (const float* q, float* qOut), (override));

        // Advanced Quaternion Algebra
        MOCK_METHOD(void, quatAverage, (const float* q1, const float* q2, float* qOut), (override));
        MOCK_METHOD(void, quatExponential, (const float* rotVec3, float* qOut), (override));
        MOCK_METHOD(float, quatAngularDistanceDeg, (const float* qTrue, const float* qEst), (override));

        // Frame Rotations
        MOCK_METHOD(void, quatToRotationMatrix, (const float* q, float* mat3x3), (override));
        MOCK_METHOD(void, quatToRotationMatrixInverse, (const float* q, float* mat3x3), (override));
        MOCK_METHOD(void, quatRotateVector, (const float* q, const float* v3, float* vOut3), (override));

        // Conversions
        MOCK_METHOD(void, quatToEuler, (const float* q, float* euler3), (override));
};
