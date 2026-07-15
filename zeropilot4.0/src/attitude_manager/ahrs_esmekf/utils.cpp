#include "utils.hpp"

const float32_t IDENTITY_QUATERNION[4] = {1.0f, 0.0f, 0.0f, 0.0f};
const float32_t GRAVITY_INERTIAL[3] = {0.0f, 0.0f, 9.81f};
const float32_t MAGNETOMETER_INERTIAL[3] = {1.0f, 0.0f, 0.0f};

bool normalizeVector(const float32_t *vIn, float32_t *vOut, uint32_t length) {
    float32_t normSq = 0.0f;
    arm_dot_prod_f32(vIn, vIn, length, &normSq);

    if (normSq <= 0.0f) {
        for (uint32_t i = 0; i < length; ++i) {
            vOut[i] = 0.0f;
        }

        return false;
    }

    float32_t norm = 0.0f;
    if (arm_sqrt_f32(normSq, &norm) != ARM_MATH_SUCCESS || norm < 1.0e-12f) {
        for (uint32_t i = 0; i < length; ++i) {
            vOut[i] = 0.0f;
        }

        return false;
    }

    const float32_t INV_NORM = 1.0f / norm;

    for (uint32_t i = 0; i < length; ++i) {
        vOut[i] = vIn[i] * INV_NORM;
    }

    return true;
}

void skewSymmetric(const float32_t *vIn, float32_t *sOut) {
    /*
     * S(v) = [  0, -vz,  vy
     *           vz,  0, -vx
     *          -vy, vx,   0 ]
     *
     * sOut is row-major 3x3.
     */
    sOut[0] = 0.0f;
    sOut[1] = -vIn[2];
    sOut[2] = vIn[1];

    sOut[3] = vIn[2];
    sOut[4] = 0.0f;
    sOut[5] = -vIn[0];

    sOut[6] = -vIn[1];
    sOut[7] = vIn[0];
    sOut[8] = 0.0f;
}

void ensureSymmetricMatrix(const float32_t *aIn, float32_t *aOut, uint32_t rows, uint32_t cols) {
    /*
     * Equivalent to Python:
     * A_sym = 0.5 * (A + A.T)
     *
     * For the EKF covariance P, rows and cols should both be 9.
     */
    if (rows != cols) {
        return;
    }

    for (uint32_t r = 0; r < rows; ++r) {
        for (uint32_t c = 0; c < cols; ++c) {
            const uint32_t INDEX_RC = r * cols + c;
            const uint32_t INDEX_CR = c * cols + r;

            aOut[INDEX_RC] = 0.5f * (aIn[INDEX_RC] + aIn[INDEX_CR]);
        }
    }
}

void copyVector(const float32_t *vIn, float32_t *vOut, uint32_t length) {
    for (uint32_t i = 0; i < length; ++i) {
        vOut[i] = vIn[i];
    }
}

void bToIFrameRotMatrix(const float32_t *qIn, float32_t *cOut) {
    float32_t q[4];
    normalizeQuaternion(qIn, q);

    float32_t w = q[0];
    float32_t x = q[1];
    float32_t y = q[2];
    float32_t z = q[3];

    cOut[0] = 1.0f - 2.0f * y * y - 2.0f * z * z;
    cOut[1] = 2.0f * x * y - 2.0f * z * w;
    cOut[2] = 2.0f * x * z + 2.0f * y * w;
    cOut[3] = 2.0f * x * y + 2.0f * z * w;
    cOut[4] = 1.0f - 2.0f * x * x - 2.0f * z * z;
    cOut[5] = 2.0f * y * z - 2.0f * x * w;
    cOut[6] = 2.0f * x * z - 2.0f * y * w;
    cOut[7] = 2.0f * y * z + 2.0f * x * w;
    cOut[8] = 1.0f - 2.0f * x * x - 2.0f * y * y;
}

