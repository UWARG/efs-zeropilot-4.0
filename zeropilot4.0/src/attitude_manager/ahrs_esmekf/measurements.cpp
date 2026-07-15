#include "attitude_manager/ahrs_esmekf/measurements.hpp"

#include "arm_math.h"

Measurements::Measurements(
    const float32_t *gyroInitial,
    const float32_t *accelInitial,
    const float32_t *magInitial
) {
    // Populate measurements.
    if (gyroInitial != nullptr) {
        arm_copy_f32(gyroInitial, gyroPrev, VECTOR_SIZE);
        arm_copy_f32(gyroInitial, gyroNew, VECTOR_SIZE);
    } else {
        arm_fill_f32(0.0f, gyroPrev, VECTOR_SIZE);
        arm_fill_f32(0.0f, gyroNew, VECTOR_SIZE);
    }

    if (accelInitial != nullptr) {
        arm_copy_f32(accelInitial, accelPrev, VECTOR_SIZE);
        arm_copy_f32(accelInitial, accelNew, VECTOR_SIZE);
    } else {
        arm_fill_f32(0.0f, accelPrev, VECTOR_SIZE);
        arm_fill_f32(0.0f, accelNew, VECTOR_SIZE);
    }

    if (magInitial != nullptr) {
        arm_copy_f32(magInitial, magPrev, VECTOR_SIZE);
        arm_copy_f32(magInitial, magNew, VECTOR_SIZE);
    } else {
        arm_fill_f32(0.0f, magPrev, VECTOR_SIZE);
        arm_fill_f32(0.0f, magNew, VECTOR_SIZE);
    }

    arm_fill_f32(0.0f, gyroBiasAccumulated, VECTOR_SIZE);
    arm_fill_f32(0.0f, accelBiasAccumulated, VECTOR_SIZE);
    arm_fill_f32(0.0f, magBiasAccumulated, VECTOR_SIZE);

    updateAllBars();
}

void Measurements::updateGyro(const float32_t *gyroNewIn) {
    if (gyroNewIn == nullptr) {
        return;
    }

    arm_copy_f32(gyroNew, gyroPrev, VECTOR_SIZE);
    arm_sub_f32(gyroNewIn, gyroBiasAccumulated, gyroNew, VECTOR_SIZE);

    updateGyroBar();
}

void Measurements::updateAccel(const float32_t *accelNewIn) {
    if (accelNewIn == nullptr) {
        return;
    }

    arm_copy_f32(accelNew, accelPrev, VECTOR_SIZE);
    arm_sub_f32(accelNewIn, accelBiasAccumulated, accelNew, VECTOR_SIZE);

    updateAccelBar();
}

void Measurements::updateMag(const float32_t *magNewIn) {
    if (magNewIn == nullptr) {
        return;
    }

    arm_copy_f32(magNew, magPrev, VECTOR_SIZE);
    arm_sub_f32(magNewIn, magBiasAccumulated, magNew, VECTOR_SIZE);

    updateMagBar();
}

void Measurements::updateBiases(
    const float32_t *gyroBiasNew,
    const float32_t *accelBiasNew,
    const float32_t *magBiasNew
) {
    if (gyroBiasNew != nullptr) {
        arm_add_f32(
            gyroBiasAccumulated,
            gyroBiasNew,
            gyroBiasAccumulated,
            VECTOR_SIZE
        );
    }

    if (accelBiasNew != nullptr) {
        arm_add_f32(
            accelBiasAccumulated,
            accelBiasNew,
            accelBiasAccumulated,
            VECTOR_SIZE
        );
    }

    if (magBiasNew != nullptr) {
        arm_add_f32(
            magBiasAccumulated,
            magBiasNew,
            magBiasAccumulated,
            VECTOR_SIZE
        );
    }
}

void Measurements::updateGyroBar() {
    averageVector3(gyroPrev, gyroNew, gyroBar);
}

void Measurements::updateAccelBar() {
    averageVector3(accelPrev, accelNew, accelBar);
}

void Measurements::updateMagBar() {
    averageVector3(magPrev, magNew, magBar);
}

void Measurements::updateAllBars() {
    updateGyroBar();
    updateAccelBar();
    updateMagBar();
}

void Measurements::averageVector3(
    const float32_t *a,
    const float32_t *b,
    float32_t *out
) {
    float32_t temp[VECTOR_SIZE];

    arm_add_f32(a, b, temp, VECTOR_SIZE);
    arm_scale_f32(temp, 0.5f, out, VECTOR_SIZE);
}
