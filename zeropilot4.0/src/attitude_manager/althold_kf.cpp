#include "althold_kf.hpp"

AltholdKF::AltholdKF(IMathUtils* mathUtils) : 
    math(mathUtils) {}

void AltholdKF::init(AltholdConfig_t config, float initStates[4]) {
    this->config = config;
    for (int i = 0; i < STATE_SIZE; ++i) {
        states[i] = initStates[i]; // Initialize states to the provided initial values
        P[i * STATE_SIZE + i] = 1.0f; // Initialize covariance matrix to identity matrix
        F[i * STATE_SIZE + i] = 1.0f; // Initialize F to identity matrix for later use
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
    float fx[STATE_SIZE * 1];
    math->matrixMult(F, STATE_SIZE, STATE_SIZE, states, 1, fx); // F * x
    float bu[STATE_SIZE * 1];
    math->matrixMult(B, STATE_SIZE, 1, &u, 1, bu); // B * u
    math->matrixAdd(fx, bu, states, STATE_SIZE, 1); // x = F * x + B * u
    
    // Uncertainty prediction
    float Ft[STATE_SIZE * STATE_SIZE];
    math->matrixTranspose(F, STATE_SIZE, STATE_SIZE, Ft); // F^T
    float FP[STATE_SIZE * STATE_SIZE];
    math->matrixMult(F, STATE_SIZE, STATE_SIZE, P, STATE_SIZE, FP); // F * P
    float FPFt[STATE_SIZE * STATE_SIZE];
    math->matrixMult(FP, STATE_SIZE, STATE_SIZE, Ft, STATE_SIZE, FPFt); // F * P * F^T
    math->matrixAdd(FPFt, Q, P, STATE_SIZE, STATE_SIZE); // P = F * P * F^T + Q
    // NOLINTEND(readability-identifier-naming)
}

void AltholdKF::updateBarometer(float altitude) {
    baroUpdateCount++;
    update(altitude, H_BARO, config.measNoiseBarometer);
}

void AltholdKF::updateGPSAlt(float altitude) {
    gpsUpdateCount++;
    update(altitude, H_GPS_ALT, config.measNoiseGPSAlt);
}

void AltholdKF::updateGPSVel(float verticalVelocity) {
    update(verticalVelocity, H_GPS_VEL, config.measNoiseGPSVel);
}

float AltholdKF::getEstimatedAltitude() {
    return states[ALTHOLD_STATE_ALTITUDE];
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
    float Ht[STATE_SIZE * 1];
    math->matrixTranspose(H, 1, STATE_SIZE, Ht); // H^T
    float PHt[STATE_SIZE * 1];
    math->matrixMult(P, STATE_SIZE, STATE_SIZE, Ht, 1, PHt); // P * H^T
    float HPHt[1 * 1];
    math->matrixMult(H, 1, STATE_SIZE, PHt, 1, HPHt); // H * P * H^T
    float S[1 * 1];
    math->matrixAdd(HPHt, &R, S, 1, 1); // S = H * P * H^T + R

    float Hx[1 * 1];
    math->matrixMult(H, 1, STATE_SIZE, states, 1, Hx); // H * x
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
    float K[STATE_SIZE* 1];
    math->matrixMult(PHt, STATE_SIZE, 1, Sinv, 1, K); // K = P * H^T * S^-1
    float Ky[STATE_SIZE * 1];
    math->matrixMult(K, STATE_SIZE, 1, &y, 1, Ky); // K * y
    math->matrixAdd(states, Ky, states, STATE_SIZE, 1); // x = x + K * y

    // Covariance update (Joseph form)
    float I[STATE_SIZE * STATE_SIZE] = {};
    for (int i = 0; i < STATE_SIZE; ++i) {
        I[i * STATE_SIZE + i] = 1.0f;
    }
    float KH[STATE_SIZE * STATE_SIZE];
    math->matrixMult(K, STATE_SIZE, 1, H, STATE_SIZE, KH); // K * H
    float IKH[STATE_SIZE * STATE_SIZE];
    math->matrixSub(I, KH, IKH, STATE_SIZE, STATE_SIZE); // I - K * H
    float IKHP[STATE_SIZE * STATE_SIZE];
    math->matrixMult(IKH, STATE_SIZE, STATE_SIZE, P, STATE_SIZE, IKHP); // (I - K * H) * P
    float IKHt[STATE_SIZE * STATE_SIZE];
    math->matrixTranspose(IKH, STATE_SIZE, STATE_SIZE, IKHt); // (I - K * H)^T
    float IKHPIKHt[STATE_SIZE * STATE_SIZE];
    math->matrixMult(IKHP, STATE_SIZE, STATE_SIZE, IKHt, STATE_SIZE, IKHPIKHt); // (I - K * H) * P * (I - K * H)^T
    float KR[STATE_SIZE * 1];
    math->matrixMult(K, STATE_SIZE, 1, &R, 1, KR); // K * R
    float Kt[1 * STATE_SIZE];
    math->matrixTranspose(K, STATE_SIZE, 1, Kt); // K^T
    float KRKt[STATE_SIZE * STATE_SIZE];
    math->matrixMult(KR, STATE_SIZE, 1, Kt, STATE_SIZE, KRKt); // K * R * K^T
    math->matrixAdd(IKHPIKHt, KRKt, P, STATE_SIZE, STATE_SIZE); // P = (I - K * H) * P * (I - K * H)^T + K * R * K^T
    math->ensureSymmetric(P, STATE_SIZE);
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
        for (int i = 0; i < STATE_SIZE; ++i) {
            P[3 * STATE_SIZE + i] = 0.0f;  // Row 3
            P[i * STATE_SIZE + 3] = 0.0f;  // Col 3
        }
    } else {
        // Enable, give it initial uncertainty so it can be estimated again
        P[3 * STATE_SIZE + 3] = 1.0f;
    }
}
