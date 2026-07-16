#pragma once

#include "mathutils_iface.hpp"
#include "imu_datatypes.hpp"
#include <cstdint>

class Measurements {
public:
    Measurements(IMathUtils* mathUtils);

    void init(const float* gyro_init, const float* accel_init, const float* mag_init);
    
    void updateGyro(const float* gyro_new_raw);
    void updateAccel(const float* accel_new_raw);
    void updateMag(const float* mag_new_raw);
    void updateBiases(const float* gyro_bias_new, const float* accel_bias_new, const float* mag_bias_new);
    
    void getGyroBar(float* out_bar) const;
    void getAccelBar(float* out_bar) const;
    void getMagBar(float* out_bar) const;

    float gyro_prev[3];
    float gyro_new[3];
    float accel_prev[3];
    float accel_new[3];
    float mag_prev[3];
    float mag_new[3];

    float gyro_bias_accumulated[3];
    float accel_bias_accumulated[3];
    float mag_bias_accumulated[3];

private:
    IMathUtils* math;
};

class NominalState {
public:
    NominalState(IMathUtils* mathUtils);

    void init(const float* quat_init);
    void stateExtrapolation(const float* gyro_new, const float* gyro_prev, float dt);
    void correctState(const float* small_angle_error);

    float quaternion_prev[4];
    float quaternion_new[4];

private:
    IMathUtils* math;
    void extrapolateQuaternion(const float* gyro_new, const float* gyro_prev, float dt, float* q_out);
};

class AHRS_ESMEKF {
public:
    // Constants
    static constexpr uint16_t ERROR_STATE_SZ = 9;

    struct Config {
        float gyro_cov;
        float accel_cov;
        float mag_cov;
        float gyro_bias_cov;
        float accel_bias_cov;
        float accel_gate_threshold;
        float mag_gate_threshold;
        float p_init_att;
        float p_init_bias;
        float gravity_inertial[3];
        float mag_inertial[3];
    };

    AHRS_ESMEKF(IMathUtils* mathUtils);

    void init(const float* gyro_init, const float* accel_init, const float* mag_init, 
              const float* quat_init, const Config& config);

    void stateExtrapolation(const float* gyro_new, float dt);
    void correctionAccelerometer(const float* accel_new);
    void correctionMagnetometer(const float* mag_new);

    Attitude_t getAttitudeRadians() const;

    // Public state access
    Measurements meas;
    NominalState nom;
    float P[ERROR_STATE_SZ * ERROR_STATE_SZ];

private:
    IMathUtils* math;
    Config cfg;

    float gyro_cov_mat[9];
    float accel_cov_mat[9];
    float mag_cov_mat[9];
    float gyro_bias_cov_mat[9];
    float accel_bias_cov_mat[9];

    void getStateTransitionMatrix(float dt, float* Phi);
    void getErrorStateGradientMatrixF(float* F);
    void getProcessNoiseCovMatrix(float dt, float* Q);
    
    void applyUpdate(const float* y, const float* H, const float* R, float gate_threshold);
    
    // Helper to insert a 3x3 block into a 9x9 matrix
    void insertBlock3x3(float* dst9x9, const float* src3x3, uint8_t row_start, uint8_t col_start);
};
