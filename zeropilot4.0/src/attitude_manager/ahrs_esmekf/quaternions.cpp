#include "attitude_manager/ahrs_esmekf/Quaternions.hpp"
#include "arm_math.h"
#include "dsp/fast_math_functions.h"
#include "dsp/matrix_functions.h"
#include "dsp/quaternion_math_functions.h"

#include <cmath>

namespace {

constexpr float32_t EPSILON_NORM = 1.0e-9f;
constexpr float32_t EPSILON_SMALL_ANGLE = 1.0e-12f;
constexpr float32_t PI = 3.14159265358979323846f;
constexpr float32_t RAD_TO_DEG = 180.0f / PI;

void setIdentityQuaternion(float32_t *qOut) {
    qOut[0] = 1.0f;
    qOut[1] = 0.0f;
    qOut[2] = 0.0f;
    qOut[3] = 0.0f;
}

float32_t absFloat32(float32_t value) {
    return value < 0.0f ? -value : value;
}

float32_t clampFloat32(float32_t value, float32_t minValue, float32_t maxValue) {
    if (value < minValue) {
        return minValue;
    }

    if (value > maxValue) {
        return maxValue;
    }

    return value;
}

void copyQuaternion(const float32_t *qIn, float32_t *qOut) {
    arm_copy_f32(qIn, qOut, 4);
}

}

void multiplyQuaternions(const float32_t *q1, const float32_t *q2, float32_t *qOut) {
    arm_quaternion_product_single_f32(q1, q2, qOut);
}

void inverseQuaternion(const float32_t *qIn, float32_t *qOut) {
    float32_t normSq = 0.0f;
    arm_dot_prod_f32(qIn, qIn, 4, &normSq);

    if (normSq < EPSILON_NORM) {
        setIdentityQuaternion(qOut);
        return;
    }

    arm_quaternion_inverse_f32(qIn, qOut, 1);
}

void normalizeQuaternion(const float32_t *qIn, float32_t *qOut) {
    float32_t normSq = 0.0f;
    arm_dot_prod_f32(qIn, qIn, 4, &normSq);

    if (normSq < EPSILON_NORM) {
        setIdentityQuaternion(qOut);
        return;
    }

    arm_quaternion_normalize_f32(qIn, qOut, 1);
}

void averageQuaternions(const float32_t *q1, const float32_t *q2, float32_t *qOut) {
    float32_t q1Norm[4];
    float32_t q2Norm[4];

    normalizeQuaternion(q1, q1Norm);
    normalizeQuaternion(q2, q2Norm);

    /*
     * Relative rotation:
     * r = q1^-1 * q2
     */
    float32_t q1Inv[4];
    float32_t relativeRotation[4];

    inverseQuaternion(q1Norm, q1Inv);
    multiplyQuaternions(q1Inv, q2Norm, relativeRotation);

    /*
     * Ensure shortest path.
     */
    if (relativeRotation[0] < 0.0f) {
        arm_negate_f32(relativeRotation, relativeRotation, 4);
    }

    const float32_t RELATIVE_ROTATION_W = clampFloat32(relativeRotation[0], -1.0f, 1.0f);
    const float32_t MU_NORM = 2.0f * static_cast<float32_t>(std::acos(RELATIVE_ROTATION_W));

    if (MU_NORM < EPSILON_SMALL_ANGLE) {
        copyQuaternion(q1Norm, qOut);
        return;
    }

    const float32_t SIN_HALF = arm_sin_f32(MU_NORM * 0.5f);

    if (absFloat32(SIN_HALF) < EPSILON_SMALL_ANGLE) {
        copyQuaternion(q1Norm, qOut);
        return;
    }

    const float32_t MU_SCALE = MU_NORM / SIN_HALF;

    float32_t mu[3] = {
        relativeRotation[1] * MU_SCALE,
        relativeRotation[2] * MU_SCALE,
        relativeRotation[3] * MU_SCALE
    };

    float32_t axis[3];
    arm_scale_f32(mu, 1.0f / MU_NORM, axis, 3);

    const float32_t QUARTER_NORM = MU_NORM * 0.25f;

    float32_t relativeHalfStep[4];
    relativeHalfStep[0] = arm_cos_f32(QUARTER_NORM);

    const float32_t SIN_QUARTER = arm_sin_f32(QUARTER_NORM);

    relativeHalfStep[1] = axis[0] * SIN_QUARTER;
    relativeHalfStep[2] = axis[1] * SIN_QUARTER;
    relativeHalfStep[3] = axis[2] * SIN_QUARTER;

    float32_t qAvg[4];
    multiplyQuaternions(q1Norm, relativeHalfStep, qAvg);

    normalizeQuaternion(qAvg, qOut);
}

void bToIFrameRotMatrix(const float32_t *qIn, float32_t *cOut) {
    float32_t q[4];
    normalizeQuaternion(qIn, q);

    const float32_t W = q[0];
    const float32_t X = q[1];
    const float32_t Y = q[2];
    const float32_t Z = q[3];

    cOut[0] = 1.0f - 2.0f * Y * Y - 2.0f * Z * Z;
    cOut[1] = 2.0f * X * Y - 2.0f * Z * W;
    cOut[2] = 2.0f * X * Z + 2.0f * Y * W;

    cOut[3] = 2.0f * X * Y + 2.0f * Z * W;
    cOut[4] = 1.0f - 2.0f * X * X - 2.0f * Z * Z;
    cOut[5] = 2.0f * Y * Z - 2.0f * X * W;

    cOut[6] = 2.0f * X * Z - 2.0f * Y * W;
    cOut[7] = 2.0f * Y * Z + 2.0f * X * W;
    cOut[8] = 1.0f - 2.0f * X * X - 2.0f * Y * Y;
}

void iToBFrameRotMatrix(const float32_t *qIn, float32_t *cOut) {
    float32_t qInv[4];
    inverseQuaternion(qIn, qInv);

    bToIFrameRotMatrix(qInv, cOut);
}

void quaternionExponential(const float32_t *rotationVector, float32_t *qOut) {
    float32_t thetaSq = 0.0f;
    arm_dot_prod_f32(rotationVector, rotationVector, 3, &thetaSq);

    float32_t theta = 0.0f;

    if (arm_sqrt_f32(thetaSq, &theta) != ARM_MATH_SUCCESS || theta < EPSILON_SMALL_ANGLE) {
        setIdentityQuaternion(qOut);
        return;
    }

    const float32_t INV_THETA = 1.0f / theta;

    float32_t unitAxis[3];
    arm_scale_f32(rotationVector, INV_THETA, unitAxis, 3);

    const float32_t HALF_THETA = theta * 0.5f;
    const float32_t SIN_HALF_THETA = arm_sin_f32(HALF_THETA);
    const float32_t COS_HALF_THETA = arm_cos_f32(HALF_THETA);

    qOut[0] = COS_HALF_THETA;
    qOut[1] = unitAxis[0] * SIN_HALF_THETA;
    qOut[2] = unitAxis[1] * SIN_HALF_THETA;
    qOut[3] = unitAxis[2] * SIN_HALF_THETA;

    normalizeQuaternion(qOut, qOut);
}

void rotateVector(const float32_t *vIn, const float32_t *qIn, float32_t *vOut) {
    float32_t rotationMatrixData[9];
    bToIFrameRotMatrix(qIn, rotationMatrixData);

    arm_matrix_instance_f32 rotationMatrix;
    arm_matrix_instance_f32 vector;
    arm_matrix_instance_f32 result;

    arm_mat_init_f32(&rotationMatrix, 3, 3, rotationMatrixData);
    arm_mat_init_f32(&vector, 3, 1, const_cast<float32_t *>(vIn));
    arm_mat_init_f32(&result, 3, 1, vOut);

    arm_mat_mult_f32(&rotationMatrix, &vector, &result);
}

float32_t angularDistanceDegrees(const float32_t *qTrue, const float32_t *qEst) {
    float32_t qEstInv[4];
    float32_t qErr[4];

    inverseQuaternion(qEst, qEstInv);
    multiplyQuaternions(qEstInv, qTrue, qErr);
    normalizeQuaternion(qErr, qErr);

    /*
     * Enforce shortest rotation.
     */
    if (qErr[0] < 0.0f) {
        arm_negate_f32(qErr, qErr, 4);
    }

    const float32_t W = clampFloat32(qErr[0], -1.0f, 1.0f);
    const float32_t ANGLE_RAD = 2.0f * static_cast<float32_t>(std::acos(W));

    return ANGLE_RAD * RAD_TO_DEG;
}

void quatToEuler(const float32_t *qIn, float32_t *eulerOut) {
    float32_t q[4];
    normalizeQuaternion(qIn, q);

    const float32_t W = q[0];
    const float32_t X = q[1];
    const float32_t Y = q[2];
    const float32_t Z = q[3];

    /*
     * Roll: x-axis rotation.
     */
    const float32_t SINR_COSP = 2.0f * (W * X + Y * Z);
    const float32_t COSR_COSP = 1.0f - 2.0f * (X * X + Y * Y);
    const float32_t ROLL = static_cast<float32_t>(std::atan2(SINR_COSP, COSR_COSP));

    /*
     * Pitch: y-axis rotation.
     */
    const float32_t SINP = 2.0f * (W * Y - Z * X);

    float32_t pitch = 0.0f;

    if (absFloat32(SINP) >= 1.0f) {
        pitch = SINP >= 0.0f ? (PI * 0.5f) : (-PI * 0.5f);
    } else {
        pitch = static_cast<float32_t>(std::asin(SINP));
    }

    /*
     * Yaw: z-axis rotation.
     */
    const float32_t SINY_COSP = 2.0f * (W * Z + X * Y);
    const float32_t COSY_COSP = 1.0f - 2.0f * (Y * Y + Z * Z);
    const float32_t YAW = static_cast<float32_t>(std::atan2(SINY_COSP, COSY_COSP));

    eulerOut[0] = ROLL;
    eulerOut[1] = pitch;
    eulerOut[2] = YAW;
}
