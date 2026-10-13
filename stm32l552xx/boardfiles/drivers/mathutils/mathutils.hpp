#pragma once

#include "mathutils_iface.hpp"
#include "arm_math.h"

class MathUtils : public IMathUtils {
    public:
        MathUtils() = default;
        ~MathUtils() override = default;

        float dspSinf(float x) override;
        float dspCosf(float x) override;

        float vectorNorm(const float* src, uint16_t dim) override;
        ZP_Error vectorNormalize(const float* src, float* dst, uint16_t dim) override;

        ZP_Error matrixAdd(const float* srcA, const float* srcB, float* dst, uint16_t rows, uint16_t cols) override;
        ZP_Error matrixSub(const float* srcA, const float* srcB, float* dst, uint16_t rows, uint16_t cols) override;
        ZP_Error matrixMult(const float* srcA, uint16_t rowsA, uint16_t colsA, 
                        const float* srcB, uint16_t colsB, float* dst) override;
        ZP_Error matrixTranspose(const float* src, uint16_t rows, uint16_t cols, float* dst) override;
        ZP_Error matrixScale(const float* src, float scale, float* dst, uint16_t rows, uint16_t cols) override;
        
        ZP_Error matrixInverse(const float* src, uint16_t dim, float* dst) override;

        void skewSymmetric(const float* v3, float* dst3x3) override;
        ZP_Error ensureSymmetric(float* M, uint16_t dim) override;

        // Quaternion Overrides
        void quatMultiply(const float* q1, const float* q2, float* qOut) override;
        void quatInverse(const float* q, float* qOut) override;
        void quatNormalize(const float* q, float* qOut) override;
        void quatAverage(const float* q1, const float* q2, float* qOut) override;
        void quatExponential(const float* rotVec3, float* qOut) override;
        float quatAngularDistanceDeg(const float* qTrue, const float* qEst) override;
        
        void quatToRotationMatrix(const float* q, float* mat3x3) override;
        void quatToRotationMatrixInverse(const float* q, float* mat3x3) override;
        void quatRotateVector(const float* q, const float* v3, float* vOut3) override;
        
        void quatToEuler(const float* q, float* euler3) override;
};
