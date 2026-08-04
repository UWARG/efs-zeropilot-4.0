#include "althold_kf.hpp"

AltholdKF::AltholdKF(IMathUtils* mathUtils) : 
    math(mathUtils) {}

void AltholdKF::init(AltholdConfig config) {
    this->config = config;
    for (int i = 0; i < STATE_SIZE; ++i) {
        states[i] = 0.0f; // Initialize states to zero
        P[i * STATE_SIZE + i] = 1.0f; // Initialize covariance matrix to identity matrix
        F[i * STATE_SIZE + i] = 1.0f; // Initialize F to identity matrix for later use
        B[i] = 0.0f; // Initialize B to zero for later use
    }
    dtPrev = 0;
}

void AltholdKF::predict(float u, float dt) {
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
}

void AltholdKF::updateRangefinder(float altitude) {
    update(altitude, measMatRangefinder, config.measNoiseRangefinder, rangefinderRejectCount, &AltholdKF::rangefinderGatingWrapper);
}

void AltholdKF::updateBarometer(float altitude) {
    update(altitude, measMatBaro, config.measNoiseBarometer, barometerRejectCount, nullptr);
}

void AltholdKF::updateGPSAlt(float altitude) {
    update(altitude, measMatGPSAlt, config.measNoiseGPSAlt, gpsAltRejectCount, nullptr);
}

void AltholdKF::updateGPSVel(float verticalVelocity) {
    update(verticalVelocity, measMatGPSVel, config.measNoiseGPSVel, gpsVelRejectCount, nullptr);
}

float AltholdKF::getEstimatedAltitude() {
    return states[0];
}

void AltholdKF::rebuildFBQ(float dt) {
    /*
    [1, dt, -0.5*dt**2, 0, 0],
    [0, 1, -dt, 0, 0],
    [0, 0, 1, 0, 0],
    [0, 0, 0, 1, 0],
    [0, 0, 0, 0, 1]
    */
    F[1] = dt;
    F[2] = -0.5 * dt * dt;
    F[7] = -dt;
    /*
    [0.5 * dt**2],
    [dt],
    [0],
    [0],
    [0]
    */
    B[0] = 0.5 * dt * dt;
    B[1] = dt;

    /*
    [0.25*dt**4 * self.accel_var, 0.5*dt**3 * self.accel_var, 0, 0, 0],
    [0.5*dt**3 * self.accel_var, dt**2 * self.accel_var, 0, 0, 0],
    [0, 0, dt * self.accel_bias_var, 0, 0],
    [0, 0, 0, dt * self.baro_bias_var, 0],
    [0, 0, 0, 0, dt * self.h_terrain_var]
    */
    Q[0] = config.processNoiseAccel * dt * dt * dt * dt / 4.0f;
    Q[1] = config.processNoiseAccel * dt * dt * dt / 2.0f;
    Q[5] = config.processNoiseAccel * dt * dt * dt / 2.0f;
    Q[6] = config.processNoiseAccel * dt * dt;
    Q[12] = config.processNoiseBiasAccel * dt;
    Q[18] = config.processNoiseBiasBaro * dt;
    Q[24] = config.processNoiseTerrainAlt * dt;
}

void AltholdKF::update(float measurement,const float *H, float R, uint8_t &rejectCount, void (AltholdKF::*gatingWrapper)(uint8_t &rejectCount)) {
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
        if (gatingWrapper != nullptr) {
            (this->*gatingWrapper)(rejectCount);
        } else {
            rejectCount++;
        }
        return;
    }
    rejectCount = 0;

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
    float Ptemp[STATE_SIZE * STATE_SIZE];
    math->matrixAdd(IKHPIKHt, KRKt, Ptemp, STATE_SIZE, STATE_SIZE); // P = (I - K * H) * P * (I - K * H)^T + K * R * K^T
    float ptempT[STATE_SIZE * STATE_SIZE];
    math->matrixTranspose(Ptemp, STATE_SIZE, STATE_SIZE, ptempT); // Ptemp^T
    math->matrixAdd(Ptemp, ptempT, P, STATE_SIZE, STATE_SIZE); // (Ptemp + Ptemp^T)
    math->matrixScale(P, 0.5f, P, STATE_SIZE, STATE_SIZE); // P = 0.5 * (Ptemp + Ptemp^T) to ensure symmetry
    return;
}

void AltholdKF::rangefinderGatingWrapper(uint8_t &rangefinderRejectCount) {
    rangefinderRejectCount++;
    if (rangefinderRejectCount > 50) { // 0.5 seconds at 100 Hz
        // Inflate terrainAlt cov to signal that we no longer know it well
        P[23] = 10.0f;
        rangefinderRejectCount = 0;
    }
}
