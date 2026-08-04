#include "althold_kf.hpp"

static void rebuildFBQ(uint32_t dt);
static void calcInnovationCov(const float* H, const float* P, float R, float* S);

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

void AltholdKF::predict(float* u, uint32_t dt) {
    if (dtPrev != dt) { // Avoid rebuilding F, B, Q if dt hasn't changed
        rebuildFBQ(dt);
        dtPrev = dt;
    }

    // State prediction using vertical acceleration as control input
    float fx[STATE_SIZE * 1];
    math->matrixMult(F, STATE_SIZE, STATE_SIZE, states, 1, fx); // F * x
    float bu[STATE_SIZE * 1];
    math->matrixMult(B, STATE_SIZE, 1, u, 1, bu); // B * u
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
    float S[STATE_SIZE * STATE_SIZE];
    calcInnovationCov(measMatRangefinder, P, config.measNoiseRangefinder, S);

    float hx;
    math->matrixMult(measMatRangefinder, 1, STATE_SIZE, states, 1, &hx); // H * x
    // Innovation
    float y = altitude - hx; // y = z - Hx
    
    // Gating
    float nis = y * y / S[0];
    if (nis > 16.0f) {
        // Reject
        rangefinderRejectCount++;
        if (rangefinderRejectCount > 50) { // 0.5 seconds at 100 Hz
            // Inflate terrainAlt cov to signal that we no longer know it well
            P[23] = 10.0f;
            rangefinderRejectCount = 0;
        }
    } 
}

static void rebuildFBQ(uint32_t dt) {
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
    Q[11] = config.processNoiseBiasAccel * dt;
    Q[18] = config.processNoiseBiasBaro * dt;
    Q[24] = config.processNoiseTerrainAlt * dt;
}

static void calcInnovationCov(const float* H, const float* P, float R, float* S) {
    float 
}

