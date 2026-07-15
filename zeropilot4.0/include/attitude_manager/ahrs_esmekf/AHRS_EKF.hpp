#pragma once

#include "measurements.hpp"
#include "nominal_state.hpp"

#include <cstdint>

class AhrsEsMekf {
public:
    static constexpr uint32_t ERROR_STATE_SIZE = 9;
    static constexpr uint32_t VECTOR_SIZE = 3;
    static constexpr uint32_t MEASUREMENT_SIZE = 3;

    AhrsEsMekf(
        const float32_t *gyroInitial = nullptr,
        const float32_t *accelInitial = nullptr,
        const float32_t *magInitial = nullptr,
        const float32_t *quaternionInitial = nullptr,

        float32_t gyroCov = 0.0f,
        float32_t accelCov = 0.0f,
        float32_t magnetometerCov = 0.0f,
        float32_t gyroBiasCov = 0.0f,
        float32_t accelBiasCov = 0.0f,

        float32_t accelGateThreshold = 7.80f,
        float32_t magnetometerGateThreshold = 16.3f,

        float32_t pInitAtt = 0.1f,
        float32_t pInitBias = 0.01f,

        const float32_t *gravityInertialIn = nullptr,
        const float32_t *magnetometerInertialIn = nullptr
    );

    void stateExtrapolation(const float32_t *gyroNew, float32_t dt);

    bool correctionAccelerometer(const float32_t *accelerometerNew);

    bool correctionMagnetometer(const float32_t *magnetometerNew);

    Measurements measurements;
    NominalState nominalState;

    float32_t errorState[ERROR_STATE_SIZE];
    float32_t kalmanGain[ERROR_STATE_SIZE * MEASUREMENT_SIZE];

    float32_t covariance[ERROR_STATE_SIZE * ERROR_STATE_SIZE];

    float32_t gravityInertial[VECTOR_SIZE];
    float32_t magnetometerInertial[VECTOR_SIZE];

private:
    float32_t gyroCovMat[VECTOR_SIZE * VECTOR_SIZE];
    float32_t accelCovMat[VECTOR_SIZE * VECTOR_SIZE];
    float32_t magnetometerCovMat[VECTOR_SIZE * VECTOR_SIZE];
    float32_t gyroBiasCovMat[VECTOR_SIZE * VECTOR_SIZE];
    float32_t accelBiasCovMat[VECTOR_SIZE * VECTOR_SIZE];

    float32_t accelGateThreshold;
    float32_t magnetometerGateThreshold;

    void stateTransitionMatrix(float32_t dt, float32_t *phiOut);

    void errorStateGradientMatrixF(float32_t *fOut);

    void processNoiseCovMatrix(float32_t dt, float32_t *qOut);

    bool applyUpdate(
        const float32_t *y,
        const float32_t *h,
        const float32_t *r,
        float32_t gateThreshold
    );

    void setZero(float32_t *data, uint32_t length);

    void setIdentity(float32_t *data, uint32_t size);

    void setDiagonal3(float32_t *matrixOut, float32_t value);

    void copyVector3(const float32_t *in, float32_t *out);

    void copyMatrix(const float32_t *in, float32_t *out, uint32_t length);

    void symmetrizeSquareMatrixInPlace(float32_t *matrix, uint32_t size);
};
