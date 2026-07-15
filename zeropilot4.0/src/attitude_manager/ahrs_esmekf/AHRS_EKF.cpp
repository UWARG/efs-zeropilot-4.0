#include "AHRS_EKF.hpp"

#include "quaternions.hpp"
#include "utils.hpp"

AhrsEsMekf::AhrsEsMekf(
    const float32_t *gyroInitial,
    const float32_t *accelInitial,
    const float32_t *magInitial,
    const float32_t *quaternionInitial,

    float32_t gyroCov,
    float32_t accelCov,
    float32_t magnetometerCov,
    float32_t gyroBiasCov,
    float32_t accelBiasCov,

    float32_t accelGateThresholdInput,
    float32_t magnetometerGateThresholdInput,

    float32_t pInitAtt,
    float32_t pInitBias,

    const float32_t *gravityInertialIn,
    const float32_t *magnetometerInertialIn
)
    : measurements(gyroInitial, accelInitial, magInitial),
      nominalState(quaternionInitial),
      accelGateThreshold(accelGateThresholdInput),
      magnetometerGateThreshold(magnetometerGateThresholdInput) {
    setDiagonal3(gyroCovMat, gyroCov);
    setDiagonal3(accelCovMat, accelCov);
    setDiagonal3(magnetometerCovMat, magnetometerCov);
    setDiagonal3(gyroBiasCovMat, gyroBiasCov);
    setDiagonal3(accelBiasCovMat, accelBiasCov);

    setZero(errorState, ERROR_STATE_SIZE);
    setZero(kalmanGain, ERROR_STATE_SIZE * MEASUREMENT_SIZE);

    setZero(covariance, ERROR_STATE_SIZE * ERROR_STATE_SIZE);

    // covariance[0:3, 0:3] = pInitAtt * I
    // covariance[3:6, 3:6] = pInitBias * I
    // covariance[6:9, 6:9] = pInitBias * I
    for (uint32_t i = 0; i < VECTOR_SIZE; ++i) {
        covariance[i * ERROR_STATE_SIZE + i] = pInitAtt;
        covariance[(i + 3) * ERROR_STATE_SIZE + (i + 3)] = pInitBias;
        covariance[(i + 6) * ERROR_STATE_SIZE + (i + 6)] = pInitBias;
    }

    if (gravityInertialIn != nullptr) {
        copyVector3(gravityInertialIn, gravityInertial);
    } else {
        copyVector3(GRAVITY_INERTIAL, gravityInertial);
    }

    if (magnetometerInertialIn != nullptr) {
        copyVector3(magnetometerInertialIn, magnetometerInertial);
    } else {
        copyVector3(MAGNETOMETER_INERTIAL, magnetometerInertial);
    }
}

void AhrsEsMekf::stateExtrapolation(const float32_t *gyroNew, float32_t dt) {
    measurements.updateGyro(gyroNew);

    nominalState.stateExtrapolation(
        measurements.gyroNew,
        measurements.gyroPrev,
        dt
    );

    float32_t phiData[ERROR_STATE_SIZE * ERROR_STATE_SIZE];
    float32_t qData[ERROR_STATE_SIZE * ERROR_STATE_SIZE];

    stateTransitionMatrix(dt, phiData);
    processNoiseCovMatrix(dt, qData);

    // covariance = Phi * covariance * Phi^T + Q
    float32_t phiTransposeData[ERROR_STATE_SIZE * ERROR_STATE_SIZE];
    float32_t tempData[ERROR_STATE_SIZE * ERROR_STATE_SIZE];
    float32_t covarianceNewData[ERROR_STATE_SIZE * ERROR_STATE_SIZE];

    arm_matrix_instance_f32 phi;
    arm_matrix_instance_f32 phiTranspose;
    arm_matrix_instance_f32 covarianceMatrix;
    arm_matrix_instance_f32 temp;
    arm_matrix_instance_f32 covarianceNew;

    arm_mat_init_f32(&phi, ERROR_STATE_SIZE, ERROR_STATE_SIZE, phiData);
    arm_mat_init_f32(&phiTranspose, ERROR_STATE_SIZE, ERROR_STATE_SIZE, phiTransposeData);
    arm_mat_init_f32(&covarianceMatrix, ERROR_STATE_SIZE, ERROR_STATE_SIZE, covariance);
    arm_mat_init_f32(&temp, ERROR_STATE_SIZE, ERROR_STATE_SIZE, tempData);
    arm_mat_init_f32(&covarianceNew, ERROR_STATE_SIZE, ERROR_STATE_SIZE, covarianceNewData);

    arm_mat_trans_f32(&phi, &phiTranspose);
    arm_mat_mult_f32(&phi, &covarianceMatrix, &temp);
    arm_mat_mult_f32(&temp, &phiTranspose, &covarianceNew);

    arm_add_f32(covarianceNewData, qData, covariance, ERROR_STATE_SIZE * ERROR_STATE_SIZE);

    symmetrizeSquareMatrixInPlace(covariance, ERROR_STATE_SIZE);
}

bool AhrsEsMekf::correctionAccelerometer(const float32_t *accelerometerNew) {
    measurements.updateAccel(accelerometerNew);

    // accelPredicted = iToBFrameRotMatrix(q) @ -gravityInertial
    float32_t iToBRotationData[VECTOR_SIZE * VECTOR_SIZE];
    iToBFrameRotMatrix(nominalState.quaternionNew, iToBRotationData);

    float32_t negativeGravity[VECTOR_SIZE];
    arm_negate_f32(gravityInertial, negativeGravity, VECTOR_SIZE);

    float32_t accelPredicted[VECTOR_SIZE];

    arm_matrix_instance_f32 iToBRotation;
    arm_matrix_instance_f32 gravityVector;
    arm_matrix_instance_f32 accelPredictedVector;

    arm_mat_init_f32(&iToBRotation, VECTOR_SIZE, VECTOR_SIZE, iToBRotationData);
    arm_mat_init_f32(&gravityVector, VECTOR_SIZE, 1, negativeGravity);
    arm_mat_init_f32(&accelPredictedVector, VECTOR_SIZE, 1, accelPredicted);

    arm_mat_mult_f32(&iToBRotation, &gravityVector, &accelPredictedVector);

    // innovation = accelNew - accelPredicted
    float32_t innovation[MEASUREMENT_SIZE];
    arm_sub_f32(measurements.accelNew, accelPredicted, innovation, MEASUREMENT_SIZE);

    // H[0:3, 0:3] = skew(accelPredicted)
    // H[0:3, 6:9] = I
    float32_t hData[MEASUREMENT_SIZE * ERROR_STATE_SIZE];
    setZero(hData, MEASUREMENT_SIZE * ERROR_STATE_SIZE);

    float32_t skewAccelData[VECTOR_SIZE * VECTOR_SIZE];
    skewSymmetric(accelPredicted, skewAccelData);

    for (uint32_t row = 0; row < VECTOR_SIZE; ++row) {
        for (uint32_t col = 0; col < VECTOR_SIZE; ++col) {
            hData[row * ERROR_STATE_SIZE + col] =
                skewAccelData[row * VECTOR_SIZE + col];
        }
    }

    for (uint32_t i = 0; i < VECTOR_SIZE; ++i) {
        hData[i * ERROR_STATE_SIZE + (i + 6)] = 1.0f;
    }

    return applyUpdate(
        innovation,
        hData,
        accelCovMat,
        accelGateThreshold
    );
}

bool AhrsEsMekf::correctionMagnetometer(const float32_t *magnetometerNew) {
    float32_t magNormalized[VECTOR_SIZE];

    if (!normalizeVector(magnetometerNew, magNormalized, VECTOR_SIZE)) {
        return false;
    }

    measurements.updateMag(magNormalized);

    // magPredicted = normalize(iToBFrameRotMatrix(q) @ magnetometerInertial)
    float32_t iToBRotationData[VECTOR_SIZE * VECTOR_SIZE];
    iToBFrameRotMatrix(nominalState.quaternionNew, iToBRotationData);

    float32_t magPredictedRaw[VECTOR_SIZE];
    float32_t magPredicted[VECTOR_SIZE];

    arm_matrix_instance_f32 iToBRotation;
    arm_matrix_instance_f32 magInertialVector;
    arm_matrix_instance_f32 magPredictedRawVector;

    arm_mat_init_f32(&iToBRotation, VECTOR_SIZE, VECTOR_SIZE, iToBRotationData);
    arm_mat_init_f32(&magInertialVector, VECTOR_SIZE, 1, magnetometerInertial);
    arm_mat_init_f32(&magPredictedRawVector, VECTOR_SIZE, 1, magPredictedRaw);

    arm_mat_mult_f32(
        &iToBRotation,
        &magInertialVector,
        &magPredictedRawVector
    );

    if (!normalizeVector(magPredictedRaw, magPredicted, VECTOR_SIZE)) {
        return false;
    }

    // innovation = magNew - magPredicted
    float32_t innovation[MEASUREMENT_SIZE];
    arm_sub_f32(measurements.magNew, magPredicted, innovation, MEASUREMENT_SIZE);

    // H[0:3, 0:3] = skew(magPredicted)
    float32_t hData[MEASUREMENT_SIZE * ERROR_STATE_SIZE];
    setZero(hData, MEASUREMENT_SIZE * ERROR_STATE_SIZE);

    float32_t skewMagData[VECTOR_SIZE * VECTOR_SIZE];
    skewSymmetric(magPredicted, skewMagData);

    for (uint32_t row = 0; row < VECTOR_SIZE; ++row) {
        for (uint32_t col = 0; col < VECTOR_SIZE; ++col) {
            hData[row * ERROR_STATE_SIZE + col] =
                skewMagData[row * VECTOR_SIZE + col];
        }
    }

    return applyUpdate(
        innovation,
        hData,
        magnetometerCovMat,
        magnetometerGateThreshold
    );
}

void AhrsEsMekf::stateTransitionMatrix(float32_t dt, float32_t *phiOut) {
    // Phi = I + dt * F + 0.5 * dt^2 * F^2
    float32_t fData[ERROR_STATE_SIZE * ERROR_STATE_SIZE];
    float32_t fSquaredData[ERROR_STATE_SIZE * ERROR_STATE_SIZE];
    float32_t dtFData[ERROR_STATE_SIZE * ERROR_STATE_SIZE];
    float32_t halfDtSquaredFSquaredData[ERROR_STATE_SIZE * ERROR_STATE_SIZE];

    errorStateGradientMatrixF(fData);

    arm_matrix_instance_f32 fMatrix;
    arm_matrix_instance_f32 fSquaredMatrix;

    arm_mat_init_f32(&fMatrix, ERROR_STATE_SIZE, ERROR_STATE_SIZE, fData);
    arm_mat_init_f32(
        &fSquaredMatrix,
        ERROR_STATE_SIZE,
        ERROR_STATE_SIZE,
        fSquaredData
    );

    arm_mat_mult_f32(&fMatrix, &fMatrix, &fSquaredMatrix);

    arm_scale_f32(
        fData,
        dt,
        dtFData,
        ERROR_STATE_SIZE * ERROR_STATE_SIZE
    );

    arm_scale_f32(
        fSquaredData,
        0.5f * dt * dt,
        halfDtSquaredFSquaredData,
        ERROR_STATE_SIZE * ERROR_STATE_SIZE
    );

    setIdentity(phiOut, ERROR_STATE_SIZE);

    arm_add_f32(
        phiOut,
        dtFData,
        phiOut,
        ERROR_STATE_SIZE * ERROR_STATE_SIZE
    );

    arm_add_f32(
        phiOut,
        halfDtSquaredFSquaredData,
        phiOut,
        ERROR_STATE_SIZE * ERROR_STATE_SIZE
    );
}

void AhrsEsMekf::errorStateGradientMatrixF(float32_t *fOut) {
    setZero(fOut, ERROR_STATE_SIZE * ERROR_STATE_SIZE);

    // omegaMatrix = -skewSymmetric(gyroBar)
    float32_t skewGyroData[VECTOR_SIZE * VECTOR_SIZE];
    skewSymmetric(measurements.gyroBar, skewGyroData);

    for (uint32_t row = 0; row < VECTOR_SIZE; ++row) {
        for (uint32_t col = 0; col < VECTOR_SIZE; ++col) {
            fOut[row * ERROR_STATE_SIZE + col] =
                -skewGyroData[row * VECTOR_SIZE + col];
        }
    }

    // F[0:3, 3:6] = -I
    for (uint32_t i = 0; i < VECTOR_SIZE; ++i) {
        fOut[i * ERROR_STATE_SIZE + (i + 3)] = -1.0f;
    }
}

void AhrsEsMekf::processNoiseCovMatrix(float32_t dt, float32_t *qOut) {
    setZero(qOut, ERROR_STATE_SIZE * ERROR_STATE_SIZE);

    const float32_t DT_SQUARED = dt * dt;
    const float32_t DT_CUBED = DT_SQUARED * dt;

    for (uint32_t row = 0; row < VECTOR_SIZE; ++row) {
        for (uint32_t col = 0; col < VECTOR_SIZE; ++col) {
            const uint32_t INDEX_3 = row * VECTOR_SIZE + col;

            qOut[row * ERROR_STATE_SIZE + col] =
                gyroCovMat[INDEX_3] * dt +
                gyroBiasCovMat[INDEX_3] * DT_CUBED / 3.0f;

            qOut[row * ERROR_STATE_SIZE + (col + 3)] =
                -gyroBiasCovMat[INDEX_3] * DT_SQUARED / 2.0f;

            qOut[(row + 3) * ERROR_STATE_SIZE + col] =
                -gyroBiasCovMat[INDEX_3] * DT_SQUARED / 2.0f;

            qOut[(row + 3) * ERROR_STATE_SIZE + (col + 3)] =
                gyroBiasCovMat[INDEX_3] * dt;

            qOut[(row + 6) * ERROR_STATE_SIZE + (col + 6)] =
                accelBiasCovMat[INDEX_3] * dt;
        }
    }
}

bool AhrsEsMekf::applyUpdate(
    const float32_t *y,
    const float32_t *h,
    const float32_t *r,
    float32_t gateThreshold
) {
    // S = H * covariance * H^T + R
    float32_t hTransposeData[ERROR_STATE_SIZE * MEASUREMENT_SIZE];
    float32_t hpData[MEASUREMENT_SIZE * ERROR_STATE_SIZE];
    float32_t sData[MEASUREMENT_SIZE * MEASUREMENT_SIZE];
    float32_t sInverseData[MEASUREMENT_SIZE * MEASUREMENT_SIZE];

    arm_matrix_instance_f32 hMatrix;
    arm_matrix_instance_f32 hTransposeMatrix;
    arm_matrix_instance_f32 covarianceMatrix;
    arm_matrix_instance_f32 hpMatrix;
    arm_matrix_instance_f32 sMatrix;
    arm_matrix_instance_f32 sInverseMatrix;

    arm_mat_init_f32(
        &hMatrix,
        MEASUREMENT_SIZE,
        ERROR_STATE_SIZE,
        const_cast<float32_t *>(h)
    );

    arm_mat_init_f32(
        &hTransposeMatrix,
        ERROR_STATE_SIZE,
        MEASUREMENT_SIZE,
        hTransposeData
    );

    arm_mat_init_f32(&covarianceMatrix, ERROR_STATE_SIZE, ERROR_STATE_SIZE, covariance);
    arm_mat_init_f32(&hpMatrix, MEASUREMENT_SIZE, ERROR_STATE_SIZE, hpData);
    arm_mat_init_f32(&sMatrix, MEASUREMENT_SIZE, MEASUREMENT_SIZE, sData);
    arm_mat_init_f32(
        &sInverseMatrix,
        MEASUREMENT_SIZE,
        MEASUREMENT_SIZE,
        sInverseData
    );

    arm_mat_trans_f32(&hMatrix, &hTransposeMatrix);
    arm_mat_mult_f32(&hMatrix, &covarianceMatrix, &hpMatrix);
    arm_mat_mult_f32(&hpMatrix, &hTransposeMatrix, &sMatrix);

    arm_add_f32(sData, r, sData, MEASUREMENT_SIZE * MEASUREMENT_SIZE);

    if (arm_mat_inverse_f32(&sMatrix, &sInverseMatrix) != ARM_MATH_SUCCESS) {
        return false;
    }

    // Mahalanobis distance = y^T * S^-1 * y
    float32_t sInverseYData[MEASUREMENT_SIZE];

    arm_matrix_instance_f32 yMatrix;
    arm_matrix_instance_f32 sInverseYMatrix;

    arm_mat_init_f32(
        &yMatrix,
        MEASUREMENT_SIZE,
        1,
        const_cast<float32_t *>(y)
    );

    arm_mat_init_f32(
        &sInverseYMatrix,
        MEASUREMENT_SIZE,
        1,
        sInverseYData
    );

    arm_mat_mult_f32(&sInverseMatrix, &yMatrix, &sInverseYMatrix);

    float32_t mahalanobisDistance = 0.0f;
    arm_dot_prod_f32(
        y,
        sInverseYData,
        MEASUREMENT_SIZE,
        &mahalanobisDistance
    );

    if (mahalanobisDistance > gateThreshold) {
        return false;
    }

    // K = covariance * H^T * S^-1
    float32_t phTransposeData[ERROR_STATE_SIZE * MEASUREMENT_SIZE];

    arm_matrix_instance_f32 phTransposeMatrix;
    arm_matrix_instance_f32 kMatrix;

    arm_mat_init_f32(
        &phTransposeMatrix,
        ERROR_STATE_SIZE,
        MEASUREMENT_SIZE,
        phTransposeData
    );

    arm_mat_init_f32(
        &kMatrix,
        ERROR_STATE_SIZE,
        MEASUREMENT_SIZE,
        kalmanGain
    );

    arm_mat_mult_f32(&covarianceMatrix, &hTransposeMatrix, &phTransposeMatrix);
    arm_mat_mult_f32(&phTransposeMatrix, &sInverseMatrix, &kMatrix);

    // errorState = K * y
    arm_matrix_instance_f32 errorStateMatrix;

    arm_mat_init_f32(
        &errorStateMatrix,
        ERROR_STATE_SIZE,
        1,
        errorState
    );

    arm_mat_mult_f32(&kMatrix, &yMatrix, &errorStateMatrix);

    // covariance = (I - K * H) * covariance
    float32_t khData[ERROR_STATE_SIZE * ERROR_STATE_SIZE];
    float32_t iMinusKhData[ERROR_STATE_SIZE * ERROR_STATE_SIZE];
    float32_t covarianceUpdatedData[ERROR_STATE_SIZE * ERROR_STATE_SIZE];

    arm_matrix_instance_f32 khMatrix;
    arm_matrix_instance_f32 iMinusKhMatrix;
    arm_matrix_instance_f32 covarianceUpdatedMatrix;

    arm_mat_init_f32(
        &khMatrix,
        ERROR_STATE_SIZE,
        ERROR_STATE_SIZE,
        khData
    );

    arm_mat_init_f32(
        &iMinusKhMatrix,
        ERROR_STATE_SIZE,
        ERROR_STATE_SIZE,
        iMinusKhData
    );

    arm_mat_init_f32(
        &covarianceUpdatedMatrix,
        ERROR_STATE_SIZE,
        ERROR_STATE_SIZE,
        covarianceUpdatedData
    );

    arm_mat_mult_f32(&kMatrix, &hMatrix, &khMatrix);

    setIdentity(iMinusKhData, ERROR_STATE_SIZE);

    arm_sub_f32(
        iMinusKhData,
        khData,
        iMinusKhData,
        ERROR_STATE_SIZE * ERROR_STATE_SIZE
    );

    arm_mat_mult_f32(&iMinusKhMatrix, &covarianceMatrix, &covarianceUpdatedMatrix);

    copyMatrix(covarianceUpdatedData, covariance, ERROR_STATE_SIZE * ERROR_STATE_SIZE);
    symmetrizeSquareMatrixInPlace(covariance, ERROR_STATE_SIZE);

    // Inject small attitude error into nominal quaternion.
    nominalState.correctState(&errorState[0]);

    // Accumulate estimated gyro and accel bias corrections.
    measurements.updateBiases(
        &errorState[3],
        &errorState[6],
        nullptr
    );

    // Reset Jacobian:
    // J[0:3, 0:3] = I - 0.5 * skew(errorState[0:3])
    float32_t jData[ERROR_STATE_SIZE * ERROR_STATE_SIZE];
    float32_t jTransposeData[ERROR_STATE_SIZE * ERROR_STATE_SIZE];
    float32_t jpData[ERROR_STATE_SIZE * ERROR_STATE_SIZE];
    float32_t covarianceResetData[ERROR_STATE_SIZE * ERROR_STATE_SIZE];

    setIdentity(jData, ERROR_STATE_SIZE);

    float32_t skewErrorData[VECTOR_SIZE * VECTOR_SIZE];
    skewSymmetric(&errorState[0], skewErrorData);

    for (uint32_t row = 0; row < VECTOR_SIZE; ++row) {
        for (uint32_t col = 0; col < VECTOR_SIZE; ++col) {
            jData[row * ERROR_STATE_SIZE + col] -=
                0.5f * skewErrorData[row * VECTOR_SIZE + col];
        }
    }

    arm_matrix_instance_f32 jMatrix;
    arm_matrix_instance_f32 jTransposeMatrix;
    arm_matrix_instance_f32 jpMatrix;
    arm_matrix_instance_f32 covarianceResetMatrix;

    arm_mat_init_f32(&jMatrix, ERROR_STATE_SIZE, ERROR_STATE_SIZE, jData);

    arm_mat_init_f32(
        &jTransposeMatrix,
        ERROR_STATE_SIZE,
        ERROR_STATE_SIZE,
        jTransposeData
    );

    arm_mat_init_f32(&jpMatrix, ERROR_STATE_SIZE, ERROR_STATE_SIZE, jpData);

    arm_mat_init_f32(
        &covarianceResetMatrix,
        ERROR_STATE_SIZE,
        ERROR_STATE_SIZE,
        covarianceResetData
    );

    arm_mat_trans_f32(&jMatrix, &jTransposeMatrix);

    // Reinitialize covarianceMatrix so it definitely points to the updated covariance.
    arm_mat_init_f32(&covarianceMatrix, ERROR_STATE_SIZE, ERROR_STATE_SIZE, covariance);

    arm_mat_mult_f32(&jMatrix, &covarianceMatrix, &jpMatrix);
    arm_mat_mult_f32(&jpMatrix, &jTransposeMatrix, &covarianceResetMatrix);

    copyMatrix(covarianceResetData, covariance, ERROR_STATE_SIZE * ERROR_STATE_SIZE);
    symmetrizeSquareMatrixInPlace(covariance, ERROR_STATE_SIZE);

    return true;
}

void AhrsEsMekf::setZero(float32_t *data, uint32_t length) {
    arm_fill_f32(0.0f, data, length);
}

void AhrsEsMekf::setIdentity(float32_t *data, uint32_t size) {
    setZero(data, size * size);

    for (uint32_t i = 0; i < size; ++i) {
        data[i * size + i] = 1.0f;
    }
}

void AhrsEsMekf::setDiagonal3(float32_t *matrixOut, float32_t value) {
    setZero(matrixOut, VECTOR_SIZE * VECTOR_SIZE);

    matrixOut[0] = value;
    matrixOut[4] = value;
    matrixOut[8] = value;
}

void AhrsEsMekf::copyVector3(const float32_t *in, float32_t *out) {
    arm_copy_f32(in, out, VECTOR_SIZE);
}

void AhrsEsMekf::copyMatrix(const float32_t *in, float32_t *out, uint32_t length) {
    arm_copy_f32(in, out, length);
}

void AhrsEsMekf::symmetrizeSquareMatrixInPlace(float32_t *matrix, uint32_t size) {
    for (uint32_t row = 0; row < size; ++row) {
        for (uint32_t col = row + 1; col < size; ++col) {
            const uint32_t ROW_COL_INDEX = row * size + col;
            const uint32_t COL_ROW_INDEX = col * size + row;

            const float32_t AVERAGE_VALUE =
                0.5f * (matrix[ROW_COL_INDEX] + matrix[COL_ROW_INDEX]);

            matrix[ROW_COL_INDEX] = AVERAGE_VALUE;
            matrix[COL_ROW_INDEX] = AVERAGE_VALUE;
        }
    }
}
