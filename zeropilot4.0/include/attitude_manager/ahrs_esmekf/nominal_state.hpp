#pragma once

#include "arm_math.h"

#include <cstdint>

class NominalState {
public:
    static constexpr uint32_t QUATERNION_SIZE = 4;
    static constexpr uint32_t VECTOR_SIZE = 3;

    NominalState();

    explicit NominalState(const float32_t *quaternionInitial);

    void stateExtrapolation(
        const float32_t *gyroNew,
        const float32_t *gyroPrev,
        float32_t dt
    );

    void correctState(const float32_t *smallAngleError);

    float32_t quaternionPrev[QUATERNION_SIZE];
    float32_t quaternionNew[QUATERNION_SIZE];

private:
    void extrapolateQuaternion(
        const float32_t *gyroNew,
        const float32_t *gyroPrev,
        float32_t dt,
        float32_t *quaternionOut
    );

    void expOmegaMatrix(
        const float32_t *gyroBar,
        float32_t dt,
        float32_t *omegaMatrixOut
    );

    void copyQuaternion(const float32_t *qIn, float32_t *qOut);
    void setIdentityQuaternion(float32_t *qOut);
};
