#include "mathutils.hpp"
#include <cmath>
#include <algorithm>

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

float MathUtils::dspSinf(float x) {
    return arm_sin_f32(x);
}

float MathUtils::dspCosf(float x) {
    return arm_cos_f32(x);
}

float MathUtils::vectorNorm(const float* src, uint16_t dim) {
    float dot = 0.0f;
    arm_dot_prod_f32(src, src, dim, &dot);
    
    float norm = 0.0f;
    arm_sqrt_f32(dot, &norm);
    return norm;
}

bool MathUtils::vectorNormalize(const float* src, float* dst, uint16_t dim) {
    float norm = vectorNorm(src, dim);
    if (norm < 1e-7f) {
        return false; // Prevent division by zero
    }
    float invNorm = 1.0f / norm;
    arm_scale_f32(src, invNorm, dst, dim);
    return true;
}

bool MathUtils::matrixAdd(const float* srcA, const float* srcB, float* dst, uint16_t rows, uint16_t cols) {
    arm_matrix_instance_f32 A, B, Out;
    arm_mat_init_f32(&A, rows, cols, const_cast<float*>(srcA));
    arm_mat_init_f32(&B, rows, cols, const_cast<float*>(srcB));
    arm_mat_init_f32(&Out, rows, cols, dst);

    return arm_mat_add_f32(&A, &B, &Out) == ARM_MATH_SUCCESS;
}

bool MathUtils::matrixSub(const float* srcA, const float* srcB, float* dst, uint16_t rows, uint16_t cols) {
    arm_matrix_instance_f32 A, B, Out;
    arm_mat_init_f32(&A, rows, cols, const_cast<float*>(srcA));
    arm_mat_init_f32(&B, rows, cols, const_cast<float*>(srcB));
    arm_mat_init_f32(&Out, rows, cols, dst);

    return arm_mat_sub_f32(&A, &B, &Out) == ARM_MATH_SUCCESS;
}

bool MathUtils::matrixMult(const float* srcA, uint16_t rowsA, uint16_t colsA, 
                           const float* srcB, uint16_t colsB, float* dst) {
    arm_matrix_instance_f32 A, B, Out;
    arm_mat_init_f32(&A, rowsA, colsA, const_cast<float*>(srcA));
    arm_mat_init_f32(&B, colsA, colsB, const_cast<float*>(srcB)); // colsA is rowsB
    arm_mat_init_f32(&Out, rowsA, colsB, dst);

    return arm_mat_mult_f32(&A, &B, &Out) == ARM_MATH_SUCCESS;
}

bool MathUtils::matrixTranspose(const float* src, uint16_t rows, uint16_t cols, float* dst) {
    arm_matrix_instance_f32 In, Out;
    arm_mat_init_f32(&In, rows, cols, const_cast<float*>(src));
    arm_mat_init_f32(&Out, cols, rows, dst);

    return arm_mat_trans_f32(&In, &Out) == ARM_MATH_SUCCESS;
}

bool MathUtils::matrixScale(const float* src, float scale, float* dst, uint16_t rows, uint16_t cols) {
    arm_matrix_instance_f32 In, Out;
    arm_mat_init_f32(&In, rows, cols, const_cast<float*>(src));
    arm_mat_init_f32(&Out, rows, cols, dst);

    return arm_mat_scale_f32(&In, scale, &Out) == ARM_MATH_SUCCESS;
}

bool MathUtils::matrixInverse(const float* src, uint16_t dim, float* dst) {
    arm_matrix_instance_f32 In, Out;
    arm_mat_init_f32(&In, dim, dim, const_cast<float*>(src));
    arm_mat_init_f32(&Out, dim, dim, dst);

    return arm_mat_inverse_f32(&In, &Out) == ARM_MATH_SUCCESS;
}

void MathUtils::skewSymmetric(const float* v3, float* dst3x3) {
    float x = v3[0];
    float y = v3[1];
    float z = v3[2];

    dst3x3[0] = 0.0f;  dst3x3[1] = -z;    dst3x3[2] = y;
    dst3x3[3] = z;     dst3x3[4] = 0.0f;  dst3x3[5] = -x;
    dst3x3[6] = -y;    dst3x3[7] = x;     dst3x3[8] = 0.0f;
}

bool MathUtils::ensureSymmetric(float* M, uint16_t dim) {
    // Computes M = (M + M^T) / 2 in-place without needing supplementary buffers
    for (uint16_t r = 0; r < dim; ++r) {
        for (uint16_t c = r + 1; c < dim; ++c) {
            uint32_t idx1 = r * dim + c;
            uint32_t idx2 = c * dim + r;
            float val = (M[idx1] + M[idx2]) * 0.5f;
            M[idx1] = val;
            M[idx2] = val;
        }
    }
    return true;
}

void MathUtils::quatMultiply(const float* q1, const float* q2, float* qOut) {
    arm_quaternion_product_f32(q1, q2, qOut, 1);
}

void MathUtils::quatInverse(const float* q, float* qOut) {
    arm_quaternion_inverse_f32(q, qOut, 1);
}

void MathUtils::quatNormalize(const float* q, float* qOut) {
    arm_quaternion_normalize_f32(q, qOut, 1);
}

void MathUtils::quatToRotationMatrix(const float* q, float* mat3x3) {
    float qNorm[4];
    quatNormalize(q, qNorm);
    arm_quaternion2rotation_f32(qNorm, mat3x3, 1);
}

void MathUtils::quatToRotationMatrixInverse(const float* q, float* mat3x3) {
    // Inverse quaternion gives the transposed/inverse rotation matrix (C_i^b)
    float qInv[4];
    quatInverse(q, qInv);
    quatToRotationMatrix(qInv, mat3x3);
}

void MathUtils::quatRotateVector(const float* q, const float* v3, float* vOut3) {
    float R[9];
    quatToRotationMatrix(q, R);
    // Multiply R (3x3) by v3 (3x1)
    matrixMult(R, 3, 3, v3, 1, vOut3);
}

void MathUtils::quatExponential(const float* rotVec3, float* qOut) {
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

void MathUtils::quatAverage(const float* q1, const float* q2, float* qOut) {
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
    
    float r_n[4];
    r_n[0] = std::cos(halfNorm / 2.0f);
    
    float axisScale = std::sin(halfNorm / 2.0f) / muNorm;
    r_n[1] = mu[0] * axisScale;
    r_n[2] = mu[1] * axisScale;
    r_n[3] = mu[2] * axisScale;

    float qAvgUnnorm[4];
    quatMultiply(q1n, r_n, qAvgUnnorm);
    quatNormalize(qAvgUnnorm, qOut);
}

float MathUtils::quatAngularDistanceDeg(const float* qTrue, const float* qEst) {
    float qEstInv[4], qErrUnnorm[4], qErr[4];
    
    quatInverse(qEst, qEstInv);
    quatMultiply(qEstInv, qTrue, qErrUnnorm);
    quatNormalize(qErrUnnorm, qErr);

    if (qErr[0] < 0.0f) {
        qErr[0] = -qErr[0];
    }

    float w = std::fmax(-1.0f, std::fmin(1.0f, qErr[0]));
    float angleRad = 2.0f * std::acos(w);
    
    return angleRad * (180.0f / M_PI);
}

void MathUtils::quatToEuler(const float* q, float* euler3) {
    float qn[4];
    quatNormalize(q, qn);
    float w = qn[0], x = qn[1], y = qn[2], z = qn[3];

    // Roll
    float sinr_cosp = 2.0f * (w * x + y * z);
    float cosr_cosp = 1.0f - 2.0f * (x * x + y * y);
    euler3[0] = std::atan2(sinr_cosp, cosr_cosp);

    // Pitch
    float sinp = 2.0f * (w * y - z * x);
    if (std::abs(sinp) >= 1.0f) {
        euler3[1] = std::copysign(M_PI / 2.0f, sinp);
    } else {
        euler3[1] = std::asin(sinp);
    }

    // Yaw
    float siny_cosp = 2.0f * (w * z + x * y);
    float cosy_cosp = 1.0f - 2.0f * (y * y + z * z);
    euler3[2] = std::atan2(siny_cosp, cosy_cosp);
}
