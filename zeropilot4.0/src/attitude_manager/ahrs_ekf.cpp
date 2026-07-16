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

// Copy the 3x3 block at block coordinates (block_row, block_col) out of / into
// the 9x9 matrix M, so CMSIS can operate on contiguous 3x3 buffers
static void getBlock3x3(const float* M9x9, int block_row, int block_col, float* out3x3) {
    for (int r = 0; r < 3; ++r) {
        for (int c = 0; c < 3; ++c) {
            out3x3[r*3 + c] = M9x9[(block_row*3 + r) * 9 + (block_col*3 + c)];
        }
    }
}

static void setBlock3x3(float* M9x9, int block_row, int block_col, const float* src3x3) {
    for (int r = 0; r < 3; ++r) {
        for (int c = 0; c < 3; ++c) {
            M9x9[(block_row*3 + r) * 9 + (block_col*3 + c)] = src3x3[r*3 + c];
        }
    }
}

void AHRS_ESMEKF::stateExtrapolation(const float* gyro_new, float dt) {
    meas.updateGyro(gyro_new);
    nom.stateExtrapolation(meas.gyro_new, meas.gyro_prev, dt);

    // Phi = I + F*dt + 0.5*dt^2 * F@F, with F = [[-S, -I, 0], [0, 0, 0], [0, 0, 0]]
    // (S = skew(gyro_bar)). F is zero outside its top block row, so Phi differs
    // from identity only in:
    //   A = Phi[0:3, 0:3] = I - S*dt + 0.5*dt^2 * S@S
    //   B = Phi[0:3, 3:6] = -I*dt + 0.5*dt^2 * S
    float g[3];
    meas.getGyroBar(g);

    float S[9];
    math->skewSymmetric(g, S);

    float S2[9];
    math->matrixMult(S, 3, 3, S, 3, S2);

    float half_dt2 = 0.5f * dt * dt;

    float A[9], B[9], neg_dt_S[9], half_dt2_S2[9];
    math->matrixScale(S, -dt, neg_dt_S, 3, 3);
    math->matrixScale(S2, half_dt2, half_dt2_S2, 3, 3);
    math->matrixAdd(neg_dt_S, half_dt2_S2, A, 3, 3);
    A[0] += 1.0f; A[4] += 1.0f; A[8] += 1.0f;

    math->matrixScale(S, half_dt2, B, 3, 3);
    B[0] -= dt; B[4] -= dt; B[8] -= dt;

    // P = Phi @ P @ Phi^T only modifies the first block row/column of P:
    //   T_j  = A @ P[0:3, 3j:3j+3] + B @ P[3:6, 3j:3j+3]   (block row 0 of Phi @ P)
    //   P[0:3, 0:3] = T0 @ A^T + T1 @ B^T
    //   P[0:3, 3j:3j+3] = T_j and its transpose across the diagonal, for j = 1, 2
    // The lower-right 6x6 of P passes through unchanged.
    float T0[9], T1[9], T2[9];
    float* T[3] = {T0, T1, T2};
    float P0j[9], P1j[9], AP[9], BP[9];
    for (int j = 0; j < 3; ++j) {
        getBlock3x3(P, 0, j, P0j);
        getBlock3x3(P, 1, j, P1j);
        math->matrixMult(A, 3, 3, P0j, 3, AP);
        math->matrixMult(B, 3, 3, P1j, 3, BP);
        math->matrixAdd(AP, BP, T[j], 3, 3);
    }

    float A_T[9], B_T[9], P00[9];
    math->matrixTranspose(A, 3, 3, A_T);
    math->matrixTranspose(B, 3, 3, B_T);
    math->matrixMult(T0, 3, 3, A_T, 3, AP);
    math->matrixMult(T1, 3, 3, B_T, 3, BP);
    math->matrixAdd(AP, BP, P00, 3, 3);

    float T_transposed[9];
    setBlock3x3(P, 0, 0, P00);
    setBlock3x3(P, 0, 1, T1);
    setBlock3x3(P, 0, 2, T2);
    math->matrixTranspose(T1, 3, 3, T_transposed);
    setBlock3x3(P, 1, 0, T_transposed);
    math->matrixTranspose(T2, 3, 3, T_transposed);
    setBlock3x3(P, 2, 0, T_transposed);

    // P += Q; Q is nonzero only on the diagonals of its 3x3 blocks
    for (int i = 0; i < 3; ++i) {
        int r = i * 3 + i;
        float gb_cov = gyro_bias_cov_mat[r];
        float q01 = -gb_cov * (dt * dt) / 2.0f;

        P[(i)*9 + (i)]     += gyro_cov_mat[r] * dt + gb_cov * (dt * dt * dt) / 3.0f;
        P[(i)*9 + (i+3)]   += q01;
        P[(i+3)*9 + (i)]   += q01;
        P[(i+3)*9 + (i+3)] += gb_cov * dt;
        P[(i+6)*9 + (i+6)] += accel_bias_cov_mat[r] * dt;
    }

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

    // H = [skew(accel_pred), 0, I]; the identity block observes the accel bias states
    float H0[9];
    math->skewSymmetric(accel_pred, H0);

    applyUpdate(innovation, H0, true, accel_cov_mat, cfg.accel_gate_threshold);
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

    // H = [skew(mag_pred), 0, 0]; the magnetometer does not observe the bias states
    float H0[9];
    math->skewSymmetric(mag_pred, H0);

    applyUpdate(innovation, H0, false, mag_cov_mat, cfg.mag_gate_threshold);
}

void AHRS_ESMEKF::applyUpdate(const float* y, const float* H0, bool observes_accel_bias,
                              const float* R, float gate_threshold) {
    // H = [H0, 0, H2] with H2 = I for the accelerometer (which observes the
    // accel bias states) and H2 = 0 for the magnetometer, so H @ P is a single
    // block row of P's 3x3 blocks:
    //   HP_j = H0 @ P[0:3, 3j:3j+3] + H2 @ P[6:9, 3j:3j+3]
    float HP0[9], HP1[9], HP2[9];
    float* HP[3] = {HP0, HP1, HP2};
    float Pblock[9], tmp[9];
    for (int j = 0; j < 3; ++j) {
        getBlock3x3(P, 0, j, Pblock);
        math->matrixMult(H0, 3, 3, Pblock, 3, HP[j]);
        if (observes_accel_bias) {
            getBlock3x3(P, 2, j, Pblock);
            math->matrixAdd(HP[j], Pblock, HP[j], 3, 3);
        }
    }

    // 1. Mahalanobis Gating: S = H @ P @ H^T + R = HP_0 @ H0^T + HP_2 @ H2^T + R
    float H0_T[9], S[9];
    math->matrixTranspose(H0, 3, 3, H0_T);
    math->matrixMult(HP0, 3, 3, H0_T, 3, S);
    if (observes_accel_bias) {
        math->matrixAdd(S, HP2, S, 3, 3);
    }
    math->matrixAdd(S, R, S, 3, 3);

    float S_inv[9]; // 3x3
    if (!math->matrixInverse(S, 3, S_inv)) return; // Failsafe against singularity

    float y_T_Sinv[3]; // 1x3
    math->matrixMult(y, 1, 3, S_inv, 3, y_T_Sinv); // y_T is identical memory layout as y

    float mahalanobis_dist = 0;
    math->matrixMult(y_T_Sinv, 1, 3, y, 1, &mahalanobis_dist);

    if (mahalanobis_dist > gate_threshold) return;

    // 2. Kalman Update: K = P @ H^T @ S_inv; P is symmetric, so P @ H^T = (H @ P)^T
    // and each 3x3 block row of K is K_i = HP_i^T @ S_inv
    float K0[9], K1[9], K2[9];
    float* K[3] = {K0, K1, K2};
    for (int i = 0; i < 3; ++i) {
        math->matrixTranspose(HP[i], 3, 3, tmp);
        math->matrixMult(tmp, 3, 3, S_inv, 3, K[i]);
    }

    // error_state = K @ y
    float error_state[9]; // 9x1
    for (int i = 0; i < 3; ++i) {
        math->matrixMult(K[i], 3, 3, y, 1, &error_state[i*3]);
    }

    // P = (I - K @ H) @ P = P - K @ (H @ P), one 3x3 block at a time:
    // block (i, j) of K @ (H @ P) is K_i @ HP_j
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            math->matrixMult(K[i], 3, 3, HP[j], 3, tmp);
            getBlock3x3(P, i, j, Pblock);
            math->matrixSub(Pblock, tmp, Pblock, 3, 3);
            setBlock3x3(P, i, j, Pblock);
        }
    }

    // 3. Update States & Biases
    nom.correctState(&error_state[0]);
    meas.updateBiases(&error_state[3], &error_state[6], nullptr);

    // 4. Reset Error State Jacobian: P = J @ P @ J^T, with J = I except
    // J[0:3, 0:3] = I - 0.5 * skew(err), so only P's first block row/column changes:
    //   P00' = J00 @ P00 @ J00^T
    //   P0j' = J00 @ P0j and Pj0' = Pj0 @ J00^T, for j = 1, 2
    float skew_err[9], J00[9];
    math->skewSymmetric(&error_state[0], skew_err);
    math->matrixScale(skew_err, -0.5f, J00, 3, 3);
    J00[0] += 1.0f; J00[4] += 1.0f; J00[8] += 1.0f;

    float J00_T[9];
    math->matrixTranspose(J00, 3, 3, J00_T);

    getBlock3x3(P, 0, 0, Pblock);
    math->matrixMult(J00, 3, 3, Pblock, 3, tmp);
    math->matrixMult(tmp, 3, 3, J00_T, 3, Pblock);
    setBlock3x3(P, 0, 0, Pblock);

    for (int j = 1; j < 3; ++j) {
        getBlock3x3(P, 0, j, Pblock);
        math->matrixMult(J00, 3, 3, Pblock, 3, tmp);
        setBlock3x3(P, 0, j, tmp);

        getBlock3x3(P, j, 0, Pblock);
        math->matrixMult(Pblock, 3, 3, J00_T, 3, tmp);
        setBlock3x3(P, j, 0, tmp);
    }

    math->ensureSymmetric(P, 9);
}

Attitude_t AHRS_ESMEKF::getAttitudeRadians() const {
    float euler_tmp[3];
    math->quatToEuler(nom.quaternion_new, euler_tmp);

    Attitude_t att;
    att.roll  = euler_tmp[0];
    att.pitch = euler_tmp[1];
    att.yaw   = euler_tmp[2];

    return att;
}
