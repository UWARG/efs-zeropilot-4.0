#pragma once

#include <cstdint>

class IMathUtils {
    protected:
        IMathUtils() = default;

    public:
        virtual ~IMathUtils() = default;

        // virtual float dspSinf(float x) = 0;
        // virtual float dspCosf(float x) = 0;

        // --- Vector Math ---
        virtual float vectorNorm(const float* src, uint16_t dim) = 0;
        virtual bool vectorNormalize(const float* src, float* dst, uint16_t dim) = 0;

        // --- Matrix Operations ---
        virtual bool matrixAdd(const float* srcA, const float* srcB, float* dst, uint16_t rows, uint16_t cols) = 0;
        virtual bool matrixSub(const float* srcA, const float* srcB, float* dst, uint16_t rows, uint16_t cols) = 0;
        virtual bool matrixMult(const float* srcA, uint16_t rowsA, uint16_t colsA, 
                                const float* srcB, uint16_t colsB, float* dst) = 0;
        virtual bool matrixTranspose(const float* src, uint16_t rows, uint16_t cols, float* dst) = 0;
        virtual bool matrixScale(const float* src, float scale, float* dst, uint16_t rows, uint16_t cols) = 0;
        virtual bool matrixInverse(const float* src, uint16_t dim, float* dst) = 0;
        virtual void skewSymmetric(const float* v3, float* dst3x3) = 0;
        virtual bool ensureSymmetric(float* M, uint16_t dim) = 0;

        // --- Quaternion Operations (q = [w, x, y, z]) ---
        virtual void quatMultiply(const float* q1, const float* q2, float* qOut) = 0;
        virtual void quatInverse(const float* q, float* qOut) = 0;
        virtual void quatNormalize(const float* q, float* qOut) = 0;
        
        // Advanced Quaternion Algebra
        virtual void quatAverage(const float* q1, const float* q2, float* qOut) = 0;
        virtual void quatExponential(const float* rotVec3, float* qOut) = 0;
        virtual float quatAngularDistanceDeg(const float* qTrue, const float* qEst) = 0;
        
        // Frame Rotations
        virtual void quatToRotationMatrix(const float* q, float* mat3x3) = 0;        // Body to Inertial
        virtual void quatToRotationMatrixInverse(const float* q, float* mat3x3) = 0; // Inertial to Body
        virtual void quatRotateVector(const float* q, const float* v3, float* vOut3) = 0;
        
        // Conversions
        virtual void quatToEuler(const float* q, float* euler3) = 0; // [roll, pitch, yaw]
};
