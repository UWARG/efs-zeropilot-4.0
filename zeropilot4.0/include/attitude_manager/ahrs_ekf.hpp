#pragma once

#include "mathutils_iface.hpp"
#include "imu_datatypes.hpp"
#include <cstdint>

class Measurements {
public:
    Measurements(IMathUtils* mathUtils);

    void init(const float* gyroInit, const float* accelInit, const float* magInit);
    
    void updateGyro(const float* gyroNewRaw);
    void updateAccel(const float* accelNewRaw);
    void updateMag(const float* magNewRaw);
    void updateBiases(const float* gyroBiasNew, const float* accelBiasNew, const float* magBiasNew);
    
    void getGyroBar(float* outBar) const;
    void getAccelBar(float* outBar) const;
    void getMagBar(float* outBar) const;

    float gyroPrev[3];
    float gyroNew[3];
    float accelPrev[3];
    float accelNew[3];
    float magPrev[3];
    float magNew[3];

    float gyroBiasAccumulated[3];
    float accelBiasAccumulated[3];
    float magBiasAccumulated[3];

private:
    IMathUtils* math;
};

class NominalState {
public:
    NominalState(IMathUtils* mathUtils);

    void init(const float* quatInit);
    void stateExtrapolation(const float* gyroNew, const float* gyroPrev, float dt);
    void correctState(const float* smallAngleError);

    float quaternionPrev[4];
    float quaternionNew[4];

private:
    IMathUtils* math;
    void extrapolateQuaternion(const float* gyroNew, const float* gyroPrev, float dt, float* qOut);
};

class AhrsEsmEkf {
public:
    // Constants
    static constexpr uint16_t ERROR_STATE_SZ = 9;

    struct Config {
        float gyroCov;
        float accelCov;
        float magCov;
        float gyroBiasCov;
        float accelBiasCov;
        float accelGateThreshold;
        float magGateThreshold;
        float pInitAtt;
        float pInitBiasGyro;
        float pInitBiasAccel;
        float gravityInertial[3];
        float magInertial[3];
    };

    AhrsEsmEkf(IMathUtils* mathUtils);

    void init(const float* gyroInit, const float* accelInit, const float* magInit, 
              const float* quatInit, const Config& config);

    void stateExtrapolation(const float* gyroNew, float dt);
    void correctionAccelerometer(const float* accelNew);
    void correctionMagnetometer(const float* magNew);

    Attitude_t getAttitudeRadians() const;

    // Public state access
    Measurements meas;
    NominalState nom;
    float p[ERROR_STATE_SZ * ERROR_STATE_SZ];

private:
    IMathUtils* math;
    Config cfg;

    float gyroCovMat[9];
    float accelCovMat[9];
    float magCovMat[9];
    float gyroBiasCovMat[9];
    float accelBiasCovMat[9];

    // Kalman update for a measurement with jacobian H = [h0, 0, H2], where h0 is
    // 3x3 and H2 is I when the measurement observes the accel bias states, else 0
    void applyUpdate(const float* y, const float* h0, bool observesAccelBias,
                     const float* R, float gateThreshold);
};
