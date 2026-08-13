#include "althold_kf.hpp"

AltholdKF::AltholdKF(IMathUtils* mathUtils) : 
    math(mathUtils) {}

void AltholdKF::init(AltholdConfig_t config, float initStates[4]) {
    this->config = config;
    for (int i = 0; i < altholdStateCount; ++i) {
        states[i] = initStates[i]; // Initialize states to the provided initial values
        P[i * altholdStateCount + i] = 1.0f; // Initialize covariance matrix to identity matrix
        F[i * altholdStateCount + i] = 1.0f; // Initialize F to identity matrix for later use
        B[i] = 0.0f; // Initialize B to zero for later use
    }
    dtPrev = 0;
}

void AltholdKF::predict(float u, float dt) {
    // NOLINTBEGIN(readability-identifier-naming) 
    if (dtPrev != dt) { // Avoid rebuilding F, B, Q if dt hasn't changed
        rebuildFBQ(dt);
        dtPrev = dt;
    }

    // State prediction using vertical acceleration as control input
    float fx[altholdStateCount * 1];
    math->matrixMult(F, altholdStateCount, altholdStateCount, states, 1, fx); // F * x
    float bu[altholdStateCount * 1];
    math->matrixMult(B, altholdStateCount, 1, &u, 1, bu); // B * u
    math->matrixAdd(fx, bu, states, altholdStateCount, 1); // x = F * x + B * u
    
    // Uncertainty prediction
    float Ft[altholdStateCount * altholdStateCount];
    math->matrixTranspose(F, altholdStateCount, altholdStateCount, Ft); // F^T
    float FP[altholdStateCount * altholdStateCount];
    math->matrixMult(F, altholdStateCount, altholdStateCount, P, altholdStateCount, FP); // F * P
    float FPFt[altholdStateCount * altholdStateCount];
    math->matrixMult(FP, altholdStateCount, altholdStateCount, Ft, altholdStateCount, FPFt); // F * P * F^T
    math->matrixAdd(FPFt, Q, P, altholdStateCount, altholdStateCount); // P = F * P * F^T + Q
    // NOLINTEND(readability-identifier-naming)
}

void AltholdKF::updateBarometer(float altitude) {
    update(altitude, H_BARO, config.measNoiseBarometer);
}

void AltholdKF::updateGPSAlt(float altitude) {
    update(altitude, H_GPS_ALT, config.measNoiseGPSAlt);
}

void AltholdKF::updateGPSVel(float verticalVelocity) {
    update(verticalVelocity, H_GPS_VEL, config.measNoiseGPSVel);
}

float AltholdKF::getEstimatedAltitude() {
    return states[altholdStateAltIdx];
}

float AltholdKF::getEstimatedVerticalVel() {
    return states[altholdStateVelIdx];
}

void AltholdKF::rebuildFBQ(float dt) {
    // NOLINTBEGIN(readability-identifier-naming)
    /*
    [1, dt, -0.5*dt**2, 0],
    [0, 1, -dt, 0],
    [0, 0, 1, 0],
    [0, 0, 0, 1]
    */
    F[1] = dt;
    F[2] = -0.5 * dt * dt;
    F[6] = -dt;
    /*
    [0.5 * dt**2],
    [dt],
    [0],
    [0]
    */
    B[0] = 0.5 * dt * dt;
    B[1] = dt;

    /*
    [0.25*dt**4 * self.accel_var, 0.5*dt**3 * self.accel_var, 0, 0],
    [0.5*dt**3 * self.accel_var, dt**2 * self.accel_var, 0, 0],
    [0, 0, dt * self.accel_bias_var, 0],
    [0, 0, 0, dt * self.baro_bias_var]
    */
    Q[0]  = config.processNoiseAccel * dt * dt * dt * dt / 4.0f;
    Q[1]  = config.processNoiseAccel * dt * dt * dt / 2.0f;
    Q[4]  = config.processNoiseAccel * dt * dt * dt / 2.0f;
    Q[5]  = config.processNoiseAccel * dt * dt;
    Q[10] = config.processNoiseBiasAccel * dt;
    Q[15] = baroBiasEnabled ? config.processNoiseBiasBaro * dt : 0.0f; 
    // NOLINTEND(readability-identifier-naming)
}

void AltholdKF::update(float measurement, const float *H, float R) {
    // NOLINTBEGIN(readability-identifier-naming)
    float Ht[altholdStateCount * 1];
    math->matrixTranspose(H, 1, altholdStateCount, Ht); // H^T
    float PHt[altholdStateCount * 1];
    math->matrixMult(P, altholdStateCount, altholdStateCount, Ht, 1, PHt); // P * H^T
    float HPHt[1 * 1];
    math->matrixMult(H, 1, altholdStateCount, PHt, 1, HPHt); // H * P * H^T
    float S[1 * 1];
    math->matrixAdd(HPHt, &R, S, 1, 1); // S = H * P * H^T + R

    float Hx[1 * 1];
    math->matrixMult(H, 1, altholdStateCount, states, 1, Hx); // H * x
    // Innovation
    float y = measurement - Hx[0]; // y = z - Hx
    
    // Gating
    float nis = y * y / S[0];
    if (nis > 16.0f) {
        // Reject
        return;
    }

    float Sinv[1 * 1];
    math->matrixInverse(S, 1, Sinv); // S^-1
    float K[altholdStateCount* 1];
    math->matrixMult(PHt, altholdStateCount, 1, Sinv, 1, K); // K = P * H^T * S^-1
    float Ky[altholdStateCount * 1];
    math->matrixMult(K, altholdStateCount, 1, &y, 1, Ky); // K * y
    math->matrixAdd(states, Ky, states, altholdStateCount, 1); // x = x + K * y

    // Covariance update (Joseph form)
    float I[altholdStateCount * altholdStateCount] = {};
    for (int i = 0; i < altholdStateCount; ++i) {
        I[i * altholdStateCount + i] = 1.0f;
    }
    float KH[altholdStateCount * altholdStateCount];
    math->matrixMult(K, altholdStateCount, 1, H, altholdStateCount, KH); // K * H
    float IKH[altholdStateCount * altholdStateCount];
    math->matrixSub(I, KH, IKH, altholdStateCount, altholdStateCount); // I - K * H
    float IKHP[altholdStateCount * altholdStateCount];
    math->matrixMult(IKH, altholdStateCount, altholdStateCount, P, altholdStateCount, IKHP); // (I - K * H) * P
    float IKHt[altholdStateCount * altholdStateCount];
    math->matrixTranspose(IKH, altholdStateCount, altholdStateCount, IKHt); // (I - K * H)^T
    float IKHPIKHt[altholdStateCount * altholdStateCount];
    math->matrixMult(IKHP, altholdStateCount, altholdStateCount, IKHt, altholdStateCount, IKHPIKHt); // (I - K * H) * P * (I - K * H)^T
    float KR[altholdStateCount * 1];
    math->matrixMult(K, altholdStateCount, 1, &R, 1, KR); // K * R
    float Kt[1 * altholdStateCount];
    math->matrixTranspose(K, altholdStateCount, 1, Kt); // K^T
    float KRKt[altholdStateCount * altholdStateCount];
    math->matrixMult(KR, altholdStateCount, 1, Kt, altholdStateCount, KRKt); // K * R * K^T
    math->matrixAdd(IKHPIKHt, KRKt, P, altholdStateCount, altholdStateCount); // P = (I - K * H) * P * (I - K * H)^T + K * R * K^T
    math->ensureSymmetric(P, altholdStateCount);
    // NOLINTEND(readability-identifier-naming)
    return;
}

void AltholdKF::setBaroBiasEnabled(bool enabled) {
    if (enabled == baroBiasEnabled) return;

    baroBiasEnabled = enabled;
    dtPrev = -1.0f;  // force rebuildFBQ() next predict so Q[15] (baro bias) is refreshed
    if (!enabled) {
        // Freeze biasBaro's covariance, zero row 3 and col 3 of P
        // This means the filter now believes it knows baro bias exactly, and will not update it anymore
        for (int i = 0; i < altholdStateCount; ++i) {
            P[3 * altholdStateCount + i] = 0.0f;  // Row 3
            P[i * altholdStateCount + 3] = 0.0f;  // Col 3
        }
    } else {
        // Enable, give it initial uncertainty so it can be estimated again
        P[3 * altholdStateCount + 3] = 1.0f;
    }
}
