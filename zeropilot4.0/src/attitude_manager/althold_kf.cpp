#include "althold_kf.hpp"

AltholdKF::AltholdKF(IMathUtils* mathUtils) : 
    math(mathUtils) {}

void AltholdKF::init(AltholdConfig_t config, float initStates[4]) {
    this->config = config;
    for (int i = 0; i < ALTHOLD_STATE_COUNT; ++i) {
        states[i] = initStates[i]; // Initialize states to the provided initial values
        P[i * ALTHOLD_STATE_COUNT + i] = 1.0f; // Initialize covariance matrix to identity matrix
        F[i * ALTHOLD_STATE_COUNT + i] = 1.0f; // Initialize F to identity matrix for later use
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
    float fx[ALTHOLD_STATE_COUNT * 1];
    math->matrixMult(F, ALTHOLD_STATE_COUNT, ALTHOLD_STATE_COUNT, states, 1, fx); // F * x
    float bu[ALTHOLD_STATE_COUNT * 1];
    math->matrixMult(B, ALTHOLD_STATE_COUNT, 1, &u, 1, bu); // B * u
    math->matrixAdd(fx, bu, states, ALTHOLD_STATE_COUNT, 1); // x = F * x + B * u
    
    // Uncertainty prediction
    float Ft[ALTHOLD_STATE_COUNT * ALTHOLD_STATE_COUNT];
    math->matrixTranspose(F, ALTHOLD_STATE_COUNT, ALTHOLD_STATE_COUNT, Ft); // F^T
    float FP[ALTHOLD_STATE_COUNT * ALTHOLD_STATE_COUNT];
    math->matrixMult(F, ALTHOLD_STATE_COUNT, ALTHOLD_STATE_COUNT, P, ALTHOLD_STATE_COUNT, FP); // F * P
    float FPFt[ALTHOLD_STATE_COUNT * ALTHOLD_STATE_COUNT];
    math->matrixMult(FP, ALTHOLD_STATE_COUNT, ALTHOLD_STATE_COUNT, Ft, ALTHOLD_STATE_COUNT, FPFt); // F * P * F^T
    math->matrixAdd(FPFt, Q, P, ALTHOLD_STATE_COUNT, ALTHOLD_STATE_COUNT); // P = F * P * F^T + Q
    // NOLINTEND(readability-identifier-naming)
}

void AltholdKF::updateBarometer(float altitude) {
    update(altitude, H_BARO, config.measNoiseBarometer, barometerRejectCount);
}

void AltholdKF::updateGPSAlt(float altitude) {
    update(altitude, H_GPS_ALT, config.measNoiseGPSAlt, gpsAltRejectCount);
}

void AltholdKF::updateGPSVel(float verticalVelocity) {
    update(verticalVelocity, H_GPS_VEL, config.measNoiseGPSVel, gpsVelRejectCount);
}

float AltholdKF::getEstimatedAltitude() {
    return states[ALTHOLD_STATE_ALT_IDX];
}

float AltholdKF::getEstimatedVerticalVel() {
    return states[ALTHOLD_STATE_VEL_IDX];
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

void AltholdKF::update(float measurement, const float *H, float R, uint8_t &rejectCount) {
    // NOLINTBEGIN(readability-identifier-naming)
    float Ht[ALTHOLD_STATE_COUNT * 1];
    math->matrixTranspose(H, 1, ALTHOLD_STATE_COUNT, Ht); // H^T
    float PHt[ALTHOLD_STATE_COUNT * 1];
    math->matrixMult(P, ALTHOLD_STATE_COUNT, ALTHOLD_STATE_COUNT, Ht, 1, PHt); // P * H^T
    float HPHt[1 * 1];
    math->matrixMult(H, 1, ALTHOLD_STATE_COUNT, PHt, 1, HPHt); // H * P * H^T
    float S[1 * 1];
    math->matrixAdd(HPHt, &R, S, 1, 1); // S = H * P * H^T + R

    float Hx[1 * 1];
    math->matrixMult(H, 1, ALTHOLD_STATE_COUNT, states, 1, Hx); // H * x
    // Innovation
    float y = measurement - Hx[0]; // y = z - Hx
    
    // Gating
    float nis = y * y / S[0];
    if (nis > NIS_GATE) {
        if (++rejectCount < MAX_CONSECUTIVE_REJECTS) {
            // Sensor glitch, reject them
            return;
        }
        rejectCount = 0;

        /*
        The sensor has disagreed for a long time, so our estimate is the one that is off, not the
        sensor. Inflate uncertainty along the measured states so the next reading passes the gate.
        */
        for (int i = 0; i < ALTHOLD_STATE_COUNT; i++) {
            // Only inflate covariance of the states that are actually measured by 
            // this sensor and dont inflate states that are delibrately frozen
            if (H[i] != 0.0f && P[i * ALTHOLD_STATE_COUNT + i] > 0.0f) {
                P[i * ALTHOLD_STATE_COUNT + i] += y * y;
            }
        }
        return;
    }
    rejectCount = 0;

    float Sinv[1 * 1];
    math->matrixInverse(S, 1, Sinv); // S^-1
    float K[ALTHOLD_STATE_COUNT* 1];
    math->matrixMult(PHt, ALTHOLD_STATE_COUNT, 1, Sinv, 1, K); // K = P * H^T * S^-1
    float Ky[ALTHOLD_STATE_COUNT * 1];
    math->matrixMult(K, ALTHOLD_STATE_COUNT, 1, &y, 1, Ky); // K * y
    math->matrixAdd(states, Ky, states, ALTHOLD_STATE_COUNT, 1); // x = x + K * y

    // Covariance update (Joseph form)
    float I[ALTHOLD_STATE_COUNT * ALTHOLD_STATE_COUNT] = {};
    for (int i = 0; i < ALTHOLD_STATE_COUNT; ++i) {
        I[i * ALTHOLD_STATE_COUNT + i] = 1.0f;
    }
    float KH[ALTHOLD_STATE_COUNT * ALTHOLD_STATE_COUNT];
    math->matrixMult(K, ALTHOLD_STATE_COUNT, 1, H, ALTHOLD_STATE_COUNT, KH); // K * H
    float IKH[ALTHOLD_STATE_COUNT * ALTHOLD_STATE_COUNT];
    math->matrixSub(I, KH, IKH, ALTHOLD_STATE_COUNT, ALTHOLD_STATE_COUNT); // I - K * H
    float IKHP[ALTHOLD_STATE_COUNT * ALTHOLD_STATE_COUNT];
    math->matrixMult(IKH, ALTHOLD_STATE_COUNT, ALTHOLD_STATE_COUNT, P, ALTHOLD_STATE_COUNT, IKHP); // (I - K * H) * P
    float IKHt[ALTHOLD_STATE_COUNT * ALTHOLD_STATE_COUNT];
    math->matrixTranspose(IKH, ALTHOLD_STATE_COUNT, ALTHOLD_STATE_COUNT, IKHt); // (I - K * H)^T
    float IKHPIKHt[ALTHOLD_STATE_COUNT * ALTHOLD_STATE_COUNT];
    math->matrixMult(IKHP, ALTHOLD_STATE_COUNT, ALTHOLD_STATE_COUNT, IKHt, ALTHOLD_STATE_COUNT, IKHPIKHt); // (I - K * H) * P * (I - K * H)^T
    float KR[ALTHOLD_STATE_COUNT * 1];
    math->matrixMult(K, ALTHOLD_STATE_COUNT, 1, &R, 1, KR); // K * R
    float Kt[1 * ALTHOLD_STATE_COUNT];
    math->matrixTranspose(K, ALTHOLD_STATE_COUNT, 1, Kt); // K^T
    float KRKt[ALTHOLD_STATE_COUNT * ALTHOLD_STATE_COUNT];
    math->matrixMult(KR, ALTHOLD_STATE_COUNT, 1, Kt, ALTHOLD_STATE_COUNT, KRKt); // K * R * K^T
    math->matrixAdd(IKHPIKHt, KRKt, P, ALTHOLD_STATE_COUNT, ALTHOLD_STATE_COUNT); // P = (I - K * H) * P * (I - K * H)^T + K * R * K^T
    math->ensureSymmetric(P, ALTHOLD_STATE_COUNT);
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
        for (int i = 0; i < ALTHOLD_STATE_COUNT; ++i) {
            P[3 * ALTHOLD_STATE_COUNT + i] = 0.0f;  // Row 3
            P[i * ALTHOLD_STATE_COUNT + 3] = 0.0f;  // Col 3
        }
    } else {
        // Enable, give it initial uncertainty so it can be estimated again
        P[3 * ALTHOLD_STATE_COUNT + 3] = 1.0f;
    }
}
