#include "ahrs_ekf.hpp"
#include <cstring>
#include <cmath>
#include <algorithm>

// ---------------------------------------------------------
// Measurements Implementation
// ---------------------------------------------------------

Measurements::Measurements(IMathUtils* mathUtils) : math(mathUtils) {
    std::memset(gyro_bias_accumulated, 0, sizeof(gyro_bias_accumulated));
    std::memset(accel_bias_accumulated, 0, sizeof(accel_bias_accumulated));
    std::memset(mag_bias_accumulated, 0, sizeof(mag_bias_accumulated));
}

void Measurements::init(const float* gyro_init, const float* accel_init, const float* mag_init) {
    std::copy(gyro_init, gyro_init + 3, gyro_prev);
    std::copy(gyro_init, gyro_init + 3, gyro_new);
    std::copy(accel_init, accel_init + 3, accel_prev);
    std::copy(accel_init, accel_init + 3, accel_new);
    std::copy(mag_init, mag_init + 3, mag_prev);
    std::copy(mag_init, mag_init + 3, mag_new);
}

void Measurements::updateGyro(const float* gyro_new_raw) {
    std::copy(gyro_new, gyro_new + 3, gyro_prev);
    for (int i = 0; i < 3; ++i) gyro_new[i] = gyro_new_raw[i] - gyro_bias_accumulated[i];
}

void Measurements::updateAccel(const float* accel_new_raw) {
    std::copy(accel_new, accel_new + 3, accel_prev);
    for (int i = 0; i < 3; ++i) accel_new[i] = accel_new_raw[i] - accel_bias_accumulated[i];
}

void Measurements::updateMag(const float* mag_new_raw) {
    std::copy(mag_new, mag_new + 3, mag_prev);
    for (int i = 0; i < 3; ++i) mag_new[i] = mag_new_raw[i] - mag_bias_accumulated[i];
}

void Measurements::updateBiases(const float* gyro_bias_new, const float* accel_bias_new, const float* mag_bias_new) {
    if (gyro_bias_new) for (int i = 0; i < 3; ++i) gyro_bias_accumulated[i] += gyro_bias_new[i];
    if (accel_bias_new) for (int i = 0; i < 3; ++i) accel_bias_accumulated[i] += accel_bias_new[i];
    if (mag_bias_new) for (int i = 0; i < 3; ++i) mag_bias_accumulated[i] += mag_bias_new[i];
}

void Measurements::getGyroBar(float* out_bar) const {
    for (int i = 0; i < 3; ++i) out_bar[i] = (gyro_prev[i] + gyro_new[i]) * 0.5f;
}

void Measurements::getAccelBar(float* out_bar) const {
    for (int i = 0; i < 3; ++i) out_bar[i] = (accel_prev[i] + accel_new[i]) * 0.5f;
}

void Measurements::getMagBar(float* out_bar) const {
    for (int i = 0; i < 3; ++i) out_bar[i] = (mag_prev[i] + mag_new[i]) * 0.5f;
}


// ---------------------------------------------------------
// NominalState Implementation
// ---------------------------------------------------------

NominalState::NominalState(IMathUtils* mathUtils) : math(mathUtils) {}

void NominalState::init(const float* quat_init) {
    math->quatNormalize(quat_init, quaternion_prev);
    std::copy(quaternion_prev, quaternion_prev + 4, quaternion_new);
}

void NominalState::stateExtrapolation(const float* gyro_new, const float* gyro_prev, float dt) {
    std::copy(quaternion_new, quaternion_new + 4, quaternion_prev);
    extrapolateQuaternion(gyro_new, gyro_prev, dt, quaternion_new);
}

void NominalState::extrapolateQuaternion(const float* gyro_new_vec, const float* gyro_prev_vec, float dt, float* q_out) {
    float gyro_bar[3];
    for (int i = 0; i < 3; ++i) gyro_bar[i] = (gyro_new_vec[i] + gyro_prev_vec[i]) * 0.5f;

    float norm_gyro = math->vectorNorm(gyro_bar, 3);
    float norm_sigma = 0.5f * dt * norm_gyro;

    float omega_matrix[16] = {0};

    if (norm_gyro < 1e-9f) {
        // Identity matrix for zero rotation
        omega_matrix[0] = 1.0f; omega_matrix[5] = 1.0f; omega_matrix[10] = 1.0f; omega_matrix[15] = 1.0f;
    } else {
        float gx = gyro_bar[0], gy = gyro_bar[1], gz = gyro_bar[2];
        float mult_matrix[16] = {
            0.0f, -gx,  -gy,  -gz,
            gx,   0.0f,  gz,  -gy,
            gy,  -gz,   0.0f,  gx,
            gz,   gy,   -gx,  0.0f
        };
        
        float cos_sig = std::cos(norm_sigma);
        float sin_sig_div = std::sin(norm_sigma) / norm_gyro;
        
        for (int i = 0; i < 16; ++i) {
            omega_matrix[i] = mult_matrix[i] * sin_sig_div;
        }
        // Add cos_sig * I
        omega_matrix[0] += cos_sig; omega_matrix[5] += cos_sig; 
        omega_matrix[10] += cos_sig; omega_matrix[15] += cos_sig;
    }

    float q_unnorm[4];
    math->matrixMult(omega_matrix, 4, 4, quaternion_prev, 1, q_unnorm);
    math->quatNormalize(q_unnorm, q_out);
}

void NominalState::correctState(const float* small_angle_error) {
    float q_err[4] = {
        1.0f, 
        0.5f * small_angle_error[0], 
        0.5f * small_angle_error[1], 
        0.5f * small_angle_error[2]
    };
    
    float q_corrected_unnorm[4];
    // Python does: q_new * q_err
    math->quatMultiply(quaternion_new, q_err, q_corrected_unnorm);
    
    math->quatNormalize(q_corrected_unnorm, quaternion_new);
    std::copy(quaternion_new, quaternion_new + 4, quaternion_prev);
}


// ---------------------------------------------------------
// AHRS_ESMEKF Implementation
// ---------------------------------------------------------

AHRS_ESMEKF::AHRS_ESMEKF(IMathUtils* mathUtils) : meas(mathUtils), nom(mathUtils), math(mathUtils) {}

void AHRS_ESMEKF::init(const float* gyro_init, const float* accel_init, const float* mag_init, 
                       const float* quat_init, const Config& config) {
    cfg = config;
    meas.init(gyro_init, accel_init, mag_init);
    nom.init(quat_init);

    // Initialize Covariance Matrices (diagonal matrices)
    std::memset(gyro_cov_mat, 0, sizeof(gyro_cov_mat));
    std::memset(accel_cov_mat, 0, sizeof(accel_cov_mat));
    std::memset(mag_cov_mat, 0, sizeof(mag_cov_mat));
    std::memset(gyro_bias_cov_mat, 0, sizeof(gyro_bias_cov_mat));
    std::memset(accel_bias_cov_mat, 0, sizeof(accel_bias_cov_mat));

    for (int i = 0; i < 3; ++i) {
        gyro_cov_mat[i*3 + i] = cfg.gyro_cov;
        accel_cov_mat[i*3 + i] = cfg.accel_cov;
        mag_cov_mat[i*3 + i] = cfg.mag_cov;
        gyro_bias_cov_mat[i*3 + i] = cfg.gyro_bias_cov;
        accel_bias_cov_mat[i*3 + i] = cfg.accel_bias_cov;
    }

    std::memset(P, 0, sizeof(P));
    for (int i = 0; i < 3; ++i) {
        P[i*9 + i] = cfg.p_init_att;             // Attitude [0:3]
        P[(i+3)*9 + (i+3)] = cfg.p_init_bias;    // Gyro Bias [3:6]
        P[(i+6)*9 + (i+6)] = cfg.p_init_bias;    // Accel Bias [6:9]
    }
}

void AHRS_ESMEKF::insertBlock3x3(float* dst9x9, const float* src3x3, uint8_t row_start, uint8_t col_start) {
    for (int r = 0; r < 3; ++r) {
        for (int c = 0; c < 3; ++c) {
            dst9x9[(row_start + r) * 9 + (col_start + c)] = src3x3[r * 3 + c];
        }
    }
}

void AHRS_ESMEKF::getErrorStateGradientMatrixF(float* F) {
    std::memset(F, 0, 81 * sizeof(float));

    float gyro_bar[3];
    meas.getGyroBar(gyro_bar);

    float skew_gyro[9];
    math->skewSymmetric(gyro_bar, skew_gyro);

    // F[0:3, 0:3] = -skew(gyro_bar)
    for (int i = 0; i < 9; ++i) skew_gyro[i] = -skew_gyro[i];
    insertBlock3x3(F, skew_gyro, 0, 0);

    // F[0:3, 3:6] = -I
    float neg_I[9] = {-1.0f, 0, 0,  0, -1.0f, 0,  0, 0, -1.0f};
    insertBlock3x3(F, neg_I, 0, 3);
}

void AHRS_ESMEKF::getStateTransitionMatrix(float dt, float* Phi) {
    float F[81];
    getErrorStateGradientMatrixF(F);

    // Phi = I + F*dt + 0.5 * dt^2 * F*F
    float F_dt[81];
    math->matrixScale(F, dt, F_dt, 9, 9);

    float F2[81];
    math->matrixMult(F, 9, 9, F, 9, F2);
    
    float F2_scaled[81];
    math->matrixScale(F2, 0.5f * dt * dt, F2_scaled, 9, 9);

    std::memset(Phi, 0, 81 * sizeof(float));
    for (int i = 0; i < 9; ++i) Phi[i*9 + i] = 1.0f; // I

    float tmp[81];
    math->matrixAdd(Phi, F_dt, tmp, 9, 9);
    math->matrixAdd(tmp, F2_scaled, Phi, 9, 9);
}

void AHRS_ESMEKF::getProcessNoiseCovMatrix(float dt, float* Q) {
    std::memset(Q, 0, 81 * sizeof(float));

    for (int i = 0; i < 3; ++i) {
        int r = i * 3 + i;
        float gb_cov = gyro_bias_cov_mat[r];
        
        float q00 = gyro_cov_mat[r] * dt + gb_cov * (dt * dt * dt) / 3.0f;
        float q01 = -gb_cov * (dt * dt) / 2.0f;
        float q11 = gb_cov * dt;
        float q22 = accel_bias_cov_mat[r] * dt;

        Q[(i)*9 + (i)] = q00;           // Q[0:3, 0:3]
        Q[(i)*9 + (i+3)] = q01;         // Q[0:3, 3:6]
        Q[(i+3)*9 + (i)] = q01;         // Q[3:6, 0:3]
        Q[(i+3)*9 + (i+3)] = q11;       // Q[3:6, 3:6]
        Q[(i+6)*9 + (i+6)] = q22;       // Q[6:9, 6:9]
    }
}

void AHRS_ESMEKF::stateExtrapolation(const float* gyro_new, float dt) {
    meas.updateGyro(gyro_new);
    nom.stateExtrapolation(meas.gyro_new, meas.gyro_prev, dt);

    float Phi[81];
    getStateTransitionMatrix(dt, Phi);

    float Q[81];
    getProcessNoiseCovMatrix(dt, Q);

    // P = Phi @ P @ Phi^T + Q
    float Phi_P[81];
    math->matrixMult(Phi, 9, 9, P, 9, Phi_P);

    float Phi_T[81];
    math->matrixTranspose(Phi, 9, 9, Phi_T);

    float Phi_P_PhiT[81];
    math->matrixMult(Phi_P, 9, 9, Phi_T, 9, Phi_P_PhiT);

    math->matrixAdd(Phi_P_PhiT, Q, P, 9, 9);
    math->ensureSymmetric(P, 9);
}

void AHRS_ESMEKF::correctionAccelerometer(const float* accel_new) {
    meas.updateAccel(accel_new);

    // accel_predicted = i_to_b_frame_rot_matrix(q_new) @ -gravity_inertial
    // i_to_b is equivalent to rotating from Inertial to Body
    float q_inv[4];
    math->quatInverse(nom.quaternion_new, q_inv);
    
    float neg_grav[3] = {-cfg.gravity_inertial[0], -cfg.gravity_inertial[1], -cfg.gravity_inertial[2]};
    float accel_pred[3];
    math->quatRotateVector(q_inv, neg_grav, accel_pred);

    float innovation[3];
    for (int i = 0; i < 3; ++i) innovation[i] = meas.accel_new[i] - accel_pred[i];

    float H[27] = {0}; // 3x9
    float skew_acc[9];
    math->skewSymmetric(accel_pred, skew_acc);
    
    // H[0:3, 0:3] = skew(accel_pred)
    for (int r = 0; r < 3; ++r) {
        for (int c = 0; c < 3; ++c) H[r * 9 + c] = skew_acc[r * 3 + c];
    }
    // H[0:3, 6:9] = I
    H[0*9 + 6] = 1.0f; H[1*9 + 7] = 1.0f; H[2*9 + 8] = 1.0f;

    applyUpdate(innovation, H, accel_cov_mat, cfg.accel_gate_threshold);
}

void AHRS_ESMEKF::correctionMagnetometer(const float* mag_new) {
    float mag_norm[3];
    math->vectorNormalize(mag_new, mag_norm, 3);
    meas.updateMag(mag_norm);

    // mag_predicted = normalize( i_to_b_rot(q_new) @ mag_inertial )
    float q_inv[4];
    math->quatInverse(nom.quaternion_new, q_inv);
    
    float mag_pred_raw[3];
    math->quatRotateVector(q_inv, cfg.mag_inertial, mag_pred_raw);
    
    float mag_pred[3];
    math->vectorNormalize(mag_pred_raw, mag_pred, 3);

    float innovation[3];
    for (int i = 0; i < 3; ++i) innovation[i] = meas.mag_new[i] - mag_pred[i];

    float H[27] = {0}; // 3x9
    float skew_mag[9];
    math->skewSymmetric(mag_pred, skew_mag);
    
    for (int r = 0; r < 3; ++r) {
        for (int c = 0; c < 3; ++c) H[r * 9 + c] = skew_mag[r * 3 + c];
    }

    applyUpdate(innovation, H, mag_cov_mat, cfg.mag_gate_threshold);
}

void AHRS_ESMEKF::applyUpdate(const float* y, const float* H, const float* R, float gate_threshold) {
    // 1. Mahalanobis Gating: S = H @ P @ H^T + R
    float H_P[27]; // 3x9
    math->matrixMult(H, 3, 9, P, 9, H_P);

    float H_T[27]; // 9x3
    math->matrixTranspose(H, 3, 9, H_T);

    float H_P_HT[9]; // 3x3
    math->matrixMult(H_P, 3, 9, H_T, 3, H_P_HT);

    float S[9]; // 3x3
    math->matrixAdd(H_P_HT, R, S, 3, 3);

    float S_inv[9]; // 3x3
    if (!math->matrixInverse(S, 3, S_inv)) return; // Failsafe against singularity

    float y_T[3]; // 1x3
    math->matrixTranspose(y, 3, 1, y_T);

    float y_T_Sinv[3]; // 1x3
    math->matrixMult(y_T, 1, 3, S_inv, 3, y_T_Sinv);

    float mahalanobis_dist = 0;
    math->matrixMult(y_T_Sinv, 1, 3, y, 1, &mahalanobis_dist);

    if (mahalanobis_dist > gate_threshold) return;

    // 2. Kalman Update: K = P @ H^T @ S_inv
    float P_HT[27]; // 9x3
    math->matrixMult(P, 9, 9, H_T, 3, P_HT);

    float K[27]; // 9x3
    math->matrixMult(P_HT, 9, 3, S_inv, 3, K);

    float error_state[9]; // 9x1
    math->matrixMult(K, 9, 3, y, 1, error_state);

    // P = (I - K @ H) @ P
    float KH[81]; // 9x9
    math->matrixMult(K, 9, 3, H, 9, KH);

    float I_KH[81];
    std::memset(I_KH, 0, sizeof(I_KH));
    for (int i = 0; i < 9; ++i) I_KH[i*9 + i] = 1.0f;
    math->matrixSub(I_KH, KH, I_KH, 9, 9);

    float P_new[81];
    math->matrixMult(I_KH, 9, 9, P, 9, P_new);
    math->ensureSymmetric(P_new, 9);
    std::copy(P_new, P_new + 81, P);

    // 3. Update States & Biases
    nom.correctState(&error_state[0]);
    meas.updateBiases(&error_state[3], &error_state[6], nullptr);

    // 4. Reset Error State Jacobian: P = J @ P @ J^T
    float J[81];
    std::memset(J, 0, sizeof(J));
    for (int i = 0; i < 9; ++i) J[i*9 + i] = 1.0f;

    float skew_err[9];
    math->skewSymmetric(&error_state[0], skew_err);
    
    // J[0:3, 0:3] = I - 0.5 * skew(err)
    for (int r = 0; r < 3; ++r) {
        for (int c = 0; c < 3; ++c) {
            J[r * 9 + c] -= 0.5f * skew_err[r * 3 + c];
        }
    }

    float J_P[81];
    math->matrixMult(J, 9, 9, P, 9, J_P);
    
    float J_T[81];
    math->matrixTranspose(J, 9, 9, J_T);
    
    math->matrixMult(J_P, 9, 9, J_T, 9, P);
    math->ensureSymmetric(P, 9);
}

Attitude_t AHRS_ESMEKF::getAttitude() const {
    float euler_tmp[3];
    math->quatToEuler(nom.quaternion_new, euler_tmp);

    Attitude_t att;
    att.roll  = euler_tmp[0];
    att.pitch = euler_tmp[1];
    att.yaw   = euler_tmp[2];

    return att;
}
