#include "attitude_manager/ahrs_esmekf/nominal_state.hpp"

#include "arm_math.h"
#include "dsp/fast_math_functions.h"
#include "dsp/matrix_functions.h"
#include "quaternions.hpp"

namespace {

constexpr float32_t SMALL_GYRO_NORM = 1.0e-9f;

}

NominalState::NominalState() {
    setIdentityQuaternion(quaternionPrev);
    setIdentityQuaternion(quaternionNew);
}

NominalState::NominalState(const float32_t *quaternionInitial) {
    if (quaternionInitial == nullptr) {
        setIdentityQuaternion(quaternionPrev);
    } else {
        normalizeQuaternion(quaternionInitial, quaternionPrev);
    }

    copyQuaternion(quaternionPrev, quaternionNew);
}

void NominalState::stateExtrapolation(
    const float32_t *gyroNew,
    const float32_t *gyroPrev,
    float32_t dt
) {
    copyQuaternion(quaternionPrev, quaternionNew);

    extrapolateQuaternion(
        gyroNew,
        gyroPrev,
        dt,
        quaternionNew
    );
}

void NominalState::extrapolateQuaternion(
    const float32_t *gyroNew,
    const float32_t *gyroPrev,
    float32_t dt,
    float32_t *quaternionOut
) {
    float32_t gyroBar[VECTOR_SIZE];

    arm_add_f32(gyroNew, gyroPrev, gyroBar, VECTOR_SIZE);
    arm_scale_f32(gyroBar, 0.5f, gyroBar, VECTOR_SIZE);

    float32_t omegaMatrixData[QUATERNION_SIZE * QUATERNION_SIZE];
    expOmegaMatrix(gyroBar, dt, omegaMatrixData);

    arm_matrix_instance_f32 omegaMatrix;
    arm_matrix_instance_f32 qPrev;
    arm_matrix_instance_f32 qNew;

    arm_mat_init_f32(&omegaMatrix, QUATERNION_SIZE, QUATERNION_SIZE, omegaMatrixData);
    arm_mat_init_f32(&qPrev, QUATERNION_SIZE, 1, quaternionPrev);
    arm_mat_init_f32(&qNew, QUATERNION_SIZE, 1, quaternionOut);

    arm_mat_mult_f32(&omegaMatrix, &qPrev, &qNew);

    normalizeQuaternion(quaternionOut, quaternionOut);
}

void NominalState::expOmegaMatrix(
    const float32_t *gyroBar,
    float32_t dt,
    float32_t *omegaMatrixOut
) {
    float32_t gyroNormSq = 0.0f;
    arm_dot_prod_f32(gyroBar, gyroBar, VECTOR_SIZE, &gyroNormSq);

    float32_t gyroNorm = 0.0f;

    if (arm_sqrt_f32(gyroNormSq, &gyroNorm) != ARM_MATH_SUCCESS || gyroNorm < SMALL_GYRO_NORM) {
        setIdentityQuaternion(omegaMatrixOut);
        return;
    }

    const float32_t SIGMA = 0.5f * dt * gyroNorm;

    const float32_t GX = gyroBar[0];
    const float32_t GY = gyroBar[1];
    const float32_t GZ = gyroBar[2];

    const float32_t GYRO_MULT_MATRIX[QUATERNION_SIZE * QUATERNION_SIZE] = {
        0.0f, -GX, -GY, -GZ,
        GX, 0.0f, GZ, -GY,
        GY, -GZ, 0.0f, GX,
        GZ, GY, -GX, 0.0f
    };

    const float32_t IDENTITY_MATRIX[QUATERNION_SIZE * QUATERNION_SIZE] = {
        1.0f, 0.0f, 0.0f, 0.0f,
        0.0f, 1.0f, 0.0f, 0.0f,
        0.0f, 0.0f, 1.0f, 0.0f,
        0.0f, 0.0f, 0.0f, 1.0f
    };

    const float32_t COS_TERM = arm_cos_f32(SIGMA);
    const float32_t SIN_TERM = arm_sin_f32(SIGMA);

    float32_t temp1[QUATERNION_SIZE * QUATERNION_SIZE];
    float32_t temp2[QUATERNION_SIZE * QUATERNION_SIZE];

    arm_scale_f32(IDENTITY_MATRIX, COS_TERM, temp1, QUATERNION_SIZE * QUATERNION_SIZE);
    arm_scale_f32(
        GYRO_MULT_MATRIX,
        SIN_TERM / gyroNorm,
        temp2,
        QUATERNION_SIZE * QUATERNION_SIZE
    );

    arm_add_f32(temp1, temp2, omegaMatrixOut, QUATERNION_SIZE * QUATERNION_SIZE);
}

void NominalState::correctState(const float32_t *smallAngleError) {
    float32_t quaternionError[QUATERNION_SIZE];

    quaternionError[0] = 1.0f;
    quaternionError[1] = 0.5f * smallAngleError[0];
    quaternionError[2] = 0.5f * smallAngleError[1];
    quaternionError[3] = 0.5f * smallAngleError[2];

    float32_t quaternionCorrected[QUATERNION_SIZE];

    multiplyQuaternions(quaternionNew, quaternionError, quaternionCorrected);
    normalizeQuaternion(quaternionCorrected, quaternionCorrected);

    copyQuaternion(quaternionCorrected, quaternionNew);
    copyQuaternion(quaternionCorrected, quaternionPrev);
}

void NominalState::copyQuaternion(const float32_t *qIn, float32_t *qOut) {
    for (uint32_t i = 0; i < QUATERNION_SIZE; ++i) {
        qOut[i] = qIn[i];
    }
}

void NominalState::setIdentityQuaternion(float32_t *qOut) {
    qOut[0] = 1.0f;
    qOut[1] = 0.0f;
    qOut[2] = 0.0f;
    qOut[3] = 0.0f;
}
