#pragma once

#include "mathutils_iface.hpp"
#include <cmath>
#include <cstring>
#include <algorithm>
#include <vector>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

/**
 * Host implementation of IMathUtils for SITL.
 *
 * The flight firmware uses a CMSIS-DSP backed MathUtils, but the SITL build
 * does not link the CMSIS-DSP matrix/quaternion sources, so this provides an
 * equivalent portable implementation. Conventions match the board driver:
 *   - matrices are row-major
 *   - quaternions are [w, x, y, z]
 *   - the rotation matrix is body-to-inertial (C_b^i)
 */
class SITL_MathUtils : public IMathUtils {
    public:
        float vectorNorm(const float* src, uint16_t dim) override {
            float dot = 0.0f;
            for (uint16_t i = 0; i < dim; ++i) dot += src[i] * src[i];
            return std::sqrt(dot);
        }

        bool vectorNormalize(const float* src, float* dst, uint16_t dim) override {
            float norm = vectorNorm(src, dim);
            if (norm < 1e-7f) return false;
            float invNorm = 1.0f / norm;
            for (uint16_t i = 0; i < dim; ++i) dst[i] = src[i] * invNorm;
            return true;
        }

        bool matrixAdd(const float* srcA, const float* srcB, float* dst, uint16_t rows, uint16_t cols) override {
            uint32_t n = static_cast<uint32_t>(rows) * cols;
            for (uint32_t i = 0; i < n; ++i) dst[i] = srcA[i] + srcB[i];
            return true;
        }

        bool matrixSub(const float* srcA, const float* srcB, float* dst, uint16_t rows, uint16_t cols) override {
            uint32_t n = static_cast<uint32_t>(rows) * cols;
            for (uint32_t i = 0; i < n; ++i) dst[i] = srcA[i] - srcB[i];
            return true;
        }

        bool matrixMult(const float* srcA, uint16_t rowsA, uint16_t colsA,
                        const float* srcB, uint16_t colsB, float* dst) override {
            // srcB is (colsA x colsB), dst is (rowsA x colsB)
            for (uint16_t r = 0; r < rowsA; ++r) {
                for (uint16_t c = 0; c < colsB; ++c) {
                    float sum = 0.0f;
                    for (uint16_t k = 0; k < colsA; ++k) {
                        sum += srcA[r * colsA + k] * srcB[k * colsB + c];
                    }
                    dst[r * colsB + c] = sum;
                }
            }
            return true;
        }

        bool matrixTranspose(const float* src, uint16_t rows, uint16_t cols, float* dst) override {
            for (uint16_t r = 0; r < rows; ++r) {
                for (uint16_t c = 0; c < cols; ++c) {
                    dst[c * rows + r] = src[r * cols + c];
                }
            }
            return true;
        }

        bool matrixScale(const float* src, float scale, float* dst, uint16_t rows, uint16_t cols) override {
            uint32_t n = static_cast<uint32_t>(rows) * cols;
            for (uint32_t i = 0; i < n; ++i) dst[i] = src[i] * scale;
            return true;
        }

        bool matrixInverse(const float* src, uint16_t dim, float* dst) override {
            // Gauss-Jordan elimination with partial pivoting on [src | I].
            std::vector<float> a(src, src + static_cast<uint32_t>(dim) * dim);
            for (uint16_t i = 0; i < dim; ++i) {
                for (uint16_t j = 0; j < dim; ++j) dst[i * dim + j] = (i == j) ? 1.0f : 0.0f;
            }

            for (uint16_t col = 0; col < dim; ++col) {
                uint16_t pivot = col;
                float best = std::fabs(a[col * dim + col]);
                for (uint16_t r = col + 1; r < dim; ++r) {
                    float v = std::fabs(a[r * dim + col]);
                    if (v > best) { best = v; pivot = r; }
                }
                if (best < 1e-12f) return false; // singular

                if (pivot != col) {
                    for (uint16_t j = 0; j < dim; ++j) {
                        std::swap(a[col * dim + j], a[pivot * dim + j]);
                        std::swap(dst[col * dim + j], dst[pivot * dim + j]);
                    }
                }

                float invPivot = 1.0f / a[col * dim + col];
                for (uint16_t j = 0; j < dim; ++j) {
                    a[col * dim + j] *= invPivot;
                    dst[col * dim + j] *= invPivot;
                }

                for (uint16_t r = 0; r < dim; ++r) {
                    if (r == col) continue;
                    float factor = a[r * dim + col];
                    if (factor == 0.0f) continue;
                    for (uint16_t j = 0; j < dim; ++j) {
                        a[r * dim + j] -= factor * a[col * dim + j];
                        dst[r * dim + j] -= factor * dst[col * dim + j];
                    }
                }
            }
            return true;
        }

        void skewSymmetric(const float* v3, float* dst3x3) override {
            float x = v3[0];
            float y = v3[1];
            float z = v3[2];

            dst3x3[0] = 0.0f;  dst3x3[1] = -z;    dst3x3[2] = y;
            dst3x3[3] = z;     dst3x3[4] = 0.0f;  dst3x3[5] = -x;
            dst3x3[6] = -y;    dst3x3[7] = x;     dst3x3[8] = 0.0f;
        }

        bool ensureSymmetric(float* m, uint16_t dim) override {
            for (uint16_t r = 0; r < dim; ++r) {
                for (uint16_t c = r + 1; c < dim; ++c) {
                    uint32_t idx1 = r * dim + c;
                    uint32_t idx2 = c * dim + r;
                    float val = (m[idx1] + m[idx2]) * 0.5f;
                    m[idx1] = val;
                    m[idx2] = val;
                }
            }
            return true;
        }

        void quatMultiply(const float* q1, const float* q2, float* qOut) override {
            qOut[0] = q1[0] * q2[0] - q1[1] * q2[1] - q1[2] * q2[2] - q1[3] * q2[3];
            qOut[1] = q1[0] * q2[1] + q1[1] * q2[0] + q1[2] * q2[3] - q1[3] * q2[2];
            qOut[2] = q1[0] * q2[2] + q1[2] * q2[0] + q1[3] * q2[1] - q1[1] * q2[3];
            qOut[3] = q1[0] * q2[3] + q1[3] * q2[0] + q1[1] * q2[2] - q1[2] * q2[1];
        }

        void quatInverse(const float* q, float* qOut) override {
            float normSq = q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3];
            if (normSq < 1e-12f) {
                qOut[0] = 1.0f; qOut[1] = 0.0f; qOut[2] = 0.0f; qOut[3] = 0.0f;
                return;
            }
            float inv = 1.0f / normSq;
            qOut[0] =  q[0] * inv;
            qOut[1] = -q[1] * inv;
            qOut[2] = -q[2] * inv;
            qOut[3] = -q[3] * inv;
        }

        void quatNormalize(const float* q, float* qOut) override {
            float norm = std::sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
            if (norm < 1e-12f) {
                qOut[0] = 1.0f; qOut[1] = 0.0f; qOut[2] = 0.0f; qOut[3] = 0.0f;
                return;
            }
            float inv = 1.0f / norm;
            qOut[0] = q[0] * inv;
            qOut[1] = q[1] * inv;
            qOut[2] = q[2] * inv;
            qOut[3] = q[3] * inv;
        }

        void quatAverage(const float* q1, const float* q2, float* qOut) override {
            float q1n[4], q2n[4], q1Inv[4], r[4];

            quatNormalize(q1, q1n);
            quatNormalize(q2, q2n);
            quatInverse(q1n, q1Inv);
            quatMultiply(q1Inv, q2n, r);

            if (r[0] < 0.0f) {
                r[0] = -r[0]; r[1] = -r[1]; r[2] = -r[2]; r[3] = -r[3];
            }

            float r0 = std::fmax(-1.0f, std::fmin(1.0f, r[0]));
            float muNorm = 2.0f * std::acos(r0);

            if (muNorm < 1e-12f) {
                std::copy(q1n, q1n + 4, qOut);
                return;
            }

            float sinHalf = std::sin(muNorm / 2.0f);
            float scale = muNorm / sinHalf;

            float mu[3] = {r[1] * scale, r[2] * scale, r[3] * scale};
            float halfNorm = muNorm / 2.0f;

            float rN[4];
            rN[0] = std::cos(halfNorm / 2.0f);

            float axisScale = std::sin(halfNorm / 2.0f) / muNorm;
            rN[1] = mu[0] * axisScale;
            rN[2] = mu[1] * axisScale;
            rN[3] = mu[2] * axisScale;

            float qAvgUnnorm[4];
            quatMultiply(q1n, rN, qAvgUnnorm);
            quatNormalize(qAvgUnnorm, qOut);
        }

        void quatExponential(const float* rotVec3, float* qOut) override {
            float theta = vectorNorm(rotVec3, 3);

            if (theta < 1e-12f) {
                qOut[0] = 1.0f; qOut[1] = 0.0f; qOut[2] = 0.0f; qOut[3] = 0.0f;
                return;
            }

            float halfTheta = theta * 0.5f;
            float sinHalf = std::sin(halfTheta);

            qOut[0] = std::cos(halfTheta);
            qOut[1] = (rotVec3[0] / theta) * sinHalf;
            qOut[2] = (rotVec3[1] / theta) * sinHalf;
            qOut[3] = (rotVec3[2] / theta) * sinHalf;
        }

        float quatAngularDistanceDeg(const float* qTrue, const float* qEst) override {
            float qEstInv[4], qErrUnnorm[4], qErr[4];

            quatInverse(qEst, qEstInv);
            quatMultiply(qEstInv, qTrue, qErrUnnorm);
            quatNormalize(qErrUnnorm, qErr);

            if (qErr[0] < 0.0f) qErr[0] = -qErr[0];

            float w = std::fmax(-1.0f, std::fmin(1.0f, qErr[0]));
            float angleRad = 2.0f * std::acos(w);

            return angleRad * (180.0f / static_cast<float>(M_PI));
        }

        void quatToRotationMatrix(const float* q, float* mat3x3) override {
            float qn[4];
            quatNormalize(q, qn);
            float w = qn[0], x = qn[1], y = qn[2], z = qn[3];

            mat3x3[0] = 1.0f - 2.0f * (y * y + z * z);
            mat3x3[1] = 2.0f * (x * y - w * z);
            mat3x3[2] = 2.0f * (x * z + w * y);
            mat3x3[3] = 2.0f * (x * y + w * z);
            mat3x3[4] = 1.0f - 2.0f * (x * x + z * z);
            mat3x3[5] = 2.0f * (y * z - w * x);
            mat3x3[6] = 2.0f * (x * z - w * y);
            mat3x3[7] = 2.0f * (y * z + w * x);
            mat3x3[8] = 1.0f - 2.0f * (x * x + y * y);
        }

        void quatToRotationMatrixInverse(const float* q, float* mat3x3) override {
            float qInv[4];
            quatInverse(q, qInv);
            quatToRotationMatrix(qInv, mat3x3);
        }

        void quatRotateVector(const float* q, const float* v3, float* vOut3) override {
            float rot[9];
            quatToRotationMatrix(q, rot);
            matrixMult(rot, 3, 3, v3, 1, vOut3);
        }

        void quatToEuler(const float* q, float* euler3) override {
            float qn[4];
            quatNormalize(q, qn);
            float w = qn[0], x = qn[1], y = qn[2], z = qn[3];

            // Roll
            float sinrCosp = 2.0f * (w * x + y * z);
            float cosrCosp = 1.0f - 2.0f * (x * x + y * y);
            euler3[0] = std::atan2(sinrCosp, cosrCosp);

            // Pitch
            float sinp = 2.0f * (w * y - z * x);
            if (std::fabs(sinp) >= 1.0f) {
                euler3[1] = std::copysign(static_cast<float>(M_PI) / 2.0f, sinp);
            } else {
                euler3[1] = std::asin(sinp);
            }

            // Yaw
            float sinyCosp = 2.0f * (w * z + x * y);
            float cosyCosp = 1.0f - 2.0f * (y * y + z * z);
            euler3[2] = std::atan2(sinyCosp, cosyCosp);
        }
};
