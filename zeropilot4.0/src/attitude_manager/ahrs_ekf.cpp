#include "ahrs_ekf.hpp"
#include <cstring>
#include <cmath>
#include <algorithm>

// ---------------------------------------------------------
// Measurements Implementation
// ---------------------------------------------------------

Measurements::Measurements(IMathUtils* mathUtils) : math(mathUtils) {
    std::memset(gyroBiasAccumulated, 0, sizeof(gyroBiasAccumulated));
    std::memset(accelBiasAccumulated, 0, sizeof(accelBiasAccumulated));
    std::memset(magBiasAccumulated, 0, sizeof(magBiasAccumulated));
}

void Measurements::init(const float* gyroInit, const float* accelInit, const float* magInit) {
    std::copy(gyroInit, gyroInit + 3, gyroPrev);
    std::copy(gyroInit, gyroInit + 3, gyroNew);
    std::copy(accelInit, accelInit + 3, accelPrev);
    std::copy(accelInit, accelInit + 3, accelNew);
    std::copy(magInit, magInit + 3, magPrev);
    std::copy(magInit, magInit + 3, magNew);
}

void Measurements::updateGyro(const float* gyroNewRaw) {
    std::copy(gyroNew, gyroNew + 3, gyroPrev);
    for (int i = 0; i < 3; ++i) gyroNew[i] = gyroNewRaw[i] - gyroBiasAccumulated[i];
}

void Measurements::updateAccel(const float* accelNewRaw) {
    std::copy(accelNew, accelNew + 3, accelPrev);
    for (int i = 0; i < 3; ++i) accelNew[i] = accelNewRaw[i] - accelBiasAccumulated[i];
}

void Measurements::updateMag(const float* magNewRaw) {
    std::copy(magNew, magNew + 3, magPrev);
    for (int i = 0; i < 3; ++i) magNew[i] = magNewRaw[i] - magBiasAccumulated[i];
}

void Measurements::updateBiases(const float* gyroBiasNew, const float* accelBiasNew, const float* magBiasNew) {
    if (gyroBiasNew) for (int i = 0; i < 3; ++i) gyroBiasAccumulated[i] += gyroBiasNew[i];
    if (accelBiasNew) for (int i = 0; i < 3; ++i) accelBiasAccumulated[i] += accelBiasNew[i];
    if (magBiasNew) for (int i = 0; i < 3; ++i) magBiasAccumulated[i] += magBiasNew[i];
}

void Measurements::getGyroBar(float* outBar) const {
    for (int i = 0; i < 3; ++i) outBar[i] = (gyroPrev[i] + gyroNew[i]) * 0.5f;
}

void Measurements::getAccelBar(float* outBar) const {
    for (int i = 0; i < 3; ++i) outBar[i] = (accelPrev[i] + accelNew[i]) * 0.5f;
}

void Measurements::getMagBar(float* outBar) const {
    for (int i = 0; i < 3; ++i) outBar[i] = (magPrev[i] + magNew[i]) * 0.5f;
}


// ---------------------------------------------------------
// NominalState Implementation
// ---------------------------------------------------------

NominalState::NominalState(IMathUtils* mathUtils) : math(mathUtils) {}

void NominalState::init(const float* quatInit) {
    math->quatNormalize(quatInit, quaternionPrev);
    std::copy(quaternionPrev, quaternionPrev + 4, quaternionNew);
}

void NominalState::stateExtrapolation(const float* gyroNew, const float* gyroPrev, float dt) {
    std::copy(quaternionNew, quaternionNew + 4, quaternionPrev);
    extrapolateQuaternion(gyroNew, gyroPrev, dt, quaternionNew);
}

void NominalState::extrapolateQuaternion(const float* gyroNewVec, const float* gyroPrevVec, float dt, float* qOut) {
    float gyroBar[3];
    for (int i = 0; i < 3; ++i) gyroBar[i] = (gyroNewVec[i] + gyroPrevVec[i]) * 0.5f;

    float normGyro = math->vectorNorm(gyroBar, 3);
    float normSigma = 0.5f * dt * normGyro;

    float omegaMatrix[16] = {0};

    if (normGyro < 1e-9f) {
        // Identity matrix for zero rotation
        omegaMatrix[0] = 1.0f; omegaMatrix[5] = 1.0f; omegaMatrix[10] = 1.0f; omegaMatrix[15] = 1.0f;
    } else {
        float gx = gyroBar[0], gy = gyroBar[1], gz = gyroBar[2];
        float multMatrix[16] = {
            0.0f, -gx,  -gy,  -gz,
            gx,   0.0f,  gz,  -gy,
            gy,  -gz,   0.0f,  gx,
            gz,   gy,   -gx,  0.0f
        };
        
        float cosSig = std::cos(normSigma);
        float sinSigDiv = std::sin(normSigma) / normGyro;
        
        for (int i = 0; i < 16; ++i) {
            omegaMatrix[i] = multMatrix[i] * sinSigDiv;
        }
        // Add cosSig * I
        omegaMatrix[0] += cosSig; omegaMatrix[5] += cosSig; 
        omegaMatrix[10] += cosSig; omegaMatrix[15] += cosSig;
    }

    float qUnnorm[4];
    math->matrixMult(omegaMatrix, 4, 4, quaternionPrev, 1, qUnnorm);
    math->quatNormalize(qUnnorm, qOut);
}

void NominalState::correctState(const float* smallAngleError) {
    float qErr[4] = {
        1.0f, 
        0.5f * smallAngleError[0], 
        0.5f * smallAngleError[1], 
        0.5f * smallAngleError[2]
    };
    
    float qCorrectedUnnorm[4];
    math->quatMultiply(quaternionNew, qErr, qCorrectedUnnorm);
    
    math->quatNormalize(qCorrectedUnnorm, quaternionNew);
    std::copy(quaternionNew, quaternionNew + 4, quaternionPrev);
}


// ---------------------------------------------------------
// AhrsEsmEkf Implementation
// AHRSEKF-----------------------------------------------

AhrsEsmEkf::AhrsEsmEkf(IMathUtils* mathUtils) : meas(mathUtils), nom(mathUtils), math(mathUtils) {}
AHRSEKFAHRSEKF
void AhrsEsmEkf::init(const float* gyroInit, const float* accelInit, const float* magInit, 
     AHRSEKF        const float* quatInit, const Config& config) {
    cfg = config;
    meas.init(gyroInit, accelInit, magInit);
    nom.init(quatInit);

    // Initialize Covariance Matrices (diagonal matrices)
    std::memset(gyroCovMat, 0, sizeof(gyroCovMat));
    std::memset(accelCovMat, 0, sizeof(accelCovMat));
    std::memset(magCovMat, 0, sizeof(magCovMat));
    std::memset(gyroBiasCovMat, 0, sizeof(gyroBiasCovMat));
    std::memset(accelBiasCovMat, 0, sizeof(accelBiasCovMat));

    for (int i = 0; i < 3; ++i) {
        gyroCovMat[i*3 + i] = cfg.gyroCov;
        accelCovMat[i*3 + i] = cfg.accelCov;
        magCovMat[i*3 + i] = cfg.magCov;
        gyroBiasCovMat[i*3 + i] = cfg.gyroBiasCov;
        accelBiasCovMat[i*3 + i] = cfg.accelBiasCov;
    }

    std::memset(p, 0, sizeof(p));
    for (int i = 0; i < 3; ++i) {
        p[i*9 + i] = cfg.pInitAtt;             // Attitude [0:3]
        p[(i+3)*9 + (i+3)] = cfg.pInitBiasGyro;    // Gyro Bias [3:6]
        p[(i+6)*9 + (i+6)] = cfg.pInitBiasAccel;    // Accel Bias [6:9]
    }
}

// Copy the 3x3 block at block coordinates (blockRow, blockCol) out of / into
// the 9x9 matrix M, so CMSIS can operate on contiguous 3x3 buffers
static void getBlock3x3(const float* m9x9, int blockRow, int blockCol, float* out3x3) {
    for (int r = 0; r < 3; ++r) {
        for (int c = 0; c < 3; ++c) {
            out3x3[r*3 + c] = m9x9[(blockRow*3 + r) * 9 + (blockCol*3 + c)];
        }
    }
}

static void setBlock3x3(float* m9x9, int blockRow, int blockCol, const float* src3x3) {
    for (int r = 0; r < 3; ++r) {
        for (int c = 0; c < 3; ++c) {
            m9x9[(blockRow*3 + r) * 9 + (blockCol*3 + c)] = src3x3[r*3 + c];
        }
    }
}

void AhrsEsmEkf::stateExtrapolation(const float* gyroNew, float dt) {
    mAHRSEKFGyro(gyroNew);
    nom.stateExtrapolation(meas.gyroNew, meas.gyroPrev, dt);
    // We only calculate the non-zero parts of the 9x9 matrix thru CMSIS DSP to optimize performance, 
    // as convention we do this by only calculating the non-zero 3x3 sub-matrices of all matrices
    // Phi = I + F*dt + 0.5*dt^2 * F@F, with F = [[-s, -I, 0], [0, 0, 0], [0, 0, 0]]
    // (s = skew(gyroBar)). F is zero outside its top block row, so Phi differs
    // from identity only in:
    //   a = Phi[0:3, 0:3] = I - s*dt + 0.5*dt^2 * s@s
    //   b = Phi[0:3, 3:6] = -I*dt + 0.5*dt^2 * s
    float g[3];
    meas.getGyroBar(g);

    float s[9];
    math->skewSymmetric(g, s);

    float s2[9];
    math->matrixMult(s, 3, 3, s, 3, s2);

    float halfDt2 = 0.5f * dt * dt;

    float a[9], b[9], negDtS[9], halfDt2S2[9];
    math->matrixScale(s, -dt, negDtS, 3, 3);
    math->matrixScale(s2, halfDt2, halfDt2S2, 3, 3);
    math->matrixAdd(negDtS, halfDt2S2, a, 3, 3);
    a[0] += 1.0f; a[4] += 1.0f; a[8] += 1.0f;

    math->matrixScale(s, halfDt2, b, 3, 3);
    b[0] -= dt; b[4] -= dt; b[8] -= dt;

    // p = Phi @ p @ Phi^t only modifies the first block row/column of p:
    //   T_j  = a @ p[0:3, 3j:3j+3] + b @ p[3:6, 3j:3j+3]   (block row 0 of Phi @ p)
    //   p[0:3, 0:3] = t0 @ a^t + t1 @ b^t
    //   p[0:3, 3j:3j+3] = T_j and its transpose across the diagonal, for j = 1, 2
    // The lower-right 6x6 of p passes through unchanged.
    float t0[9], t1[9], t2[9];
    float* t[3] = {t0, t1, t2};
    float p0j[9], p1j[9], ap[9], bp[9];
    for (int j = 0; j < 3; ++j) {
        getBlock3x3(p, 0, j, p0j);
        getBlock3x3(p, 1, j, p1j);
        math->matrixMult(a, 3, 3, p0j, 3, ap);
        math->matrixMult(b, 3, 3, p1j, 3, bp);
        math->matrixAdd(ap, bp, t[j], 3, 3);
    }

    float aT[9], bT[9], p00[9];
    math->matrixTranspose(a, 3, 3, aT);
    math->matrixTranspose(b, 3, 3, bT);
    math->matrixMult(t0, 3, 3, aT, 3, ap);
    math->matrixMult(t1, 3, 3, bT, 3, bp);
    math->matrixAdd(ap, bp, p00, 3, 3);

    float tTransposed[9];
    setBlock3x3(p, 0, 0, p00);
    setBlock3x3(p, 0, 1, t1);
    setBlock3x3(p, 0, 2, t2);
    math->matrixTranspose(t1, 3, 3, tTransposed);
    setBlock3x3(p, 1, 0, tTransposed);
    math->matrixTranspose(t2, 3, 3, tTransposed);
    setBlock3x3(p, 2, 0, tTransposed);

    // p += Q; Q is nonzero only on the diagonals of its 3x3 blocks
    for (int i = 0; i < 3; ++i) {
        int r = i * 3 + i;
        float gbCov = gyroBiasCovMat[r];
        float q01 = -gbCov * (dt * dt) / 2.0f;

        p[(i)*9 + (i)]     += gyroCovMat[r] * dt + gbCov * (dt * dt * dt) / 3.0f;
        p[(i)*9 + (i+3)]   += q01;
        p[(i+3)*9 + (i)]   += q01;
        p[(i+3)*9 + (i+3)] += gbCov * dt;
        p[(i+6)*9 + (i+6)] += accelBiasCovMat[r] * dt;
    }

    math->ensureSymmetric(p, 9);
}

void AhrsEsmEkf::correctionAccelerometer(const float* accelNew) {
    mAHRSEKFAccel(accelNew);

    // accel_predicted = i_to_b_frame_rot_matrix(q_new) @ -gravityInertial
    // i_to_b is equivalent to rotating from Inertial to Body
    float qInv[4];
    math->quatInverse(nom.quaternionNew, qInv);
    
    float negGrav[3] = {-cfg.gravityInertial[0], -cfg.gravityInertial[1], -cfg.gravityInertial[2]};
    float accelPred[3];
    math->quatRotateVector(qInv, negGrav, accelPred);

    float innovation[3];
    for (int i = 0; i < 3; ++i) innovation[i] = meas.accelNew[i] - accelPred[i];

    // H = [skew(accelPred), 0, I]; the identity block observes the accel bias states
    float h0[9];
    math->skewSymmetric(accelPred, h0);

    applyUpdate(innovation, h0, true, accelCovMat, cfg.accelGateThreshold);
}

void AhrsEsmEkf::correctionMagnetometer(const float* magNew) {
    fAHRSEKFrm[3];
    math->vectorNormalize(magNew, magNorm, 3);
    meas.updateMag(magNorm);

    // mag_predicted = normalize( i_to_b_rot(q_new) @ magInertial )
    float qInv[4];
    math->quatInverse(nom.quaternionNew, qInv);
    
    float magPredRaw[3];
    math->quatRotateVector(qInv, cfg.magInertial, magPredRaw);
    
    float magPred[3];
    math->vectorNormalize(magPredRaw, magPred, 3);

    float innovation[3];
    for (int i = 0; i < 3; ++i) innovation[i] = meas.magNew[i] - magPred[i];

    // H = [skew(magPred), 0, 0]; the magnetometer does not observe the bias states
    float h0[9];
    math->skewSymmetric(magPred, h0);

    applyUpdate(innovation, h0, false, magCovMat, cfg.magGateThreshold);
}

/*
H is assumed to be a 3x9 matrix of form [skew(accelPred), 0, I] and [skew(magPred), 0, 0], 
bc we only pass in non-zero entries of each
observesAccelBias determines which form of the two it is
*/
void AhrsEsmEkf::applyUpdate(const float* y, const float* h0, bool observesAccelBias,
     AHRSEKF               const float* R, float gateThreshold) {
    // H = [h0, 0, H2] with H2 = I for the accelerometer (which observes the
    // accel bias states) and H2 = 0 for the magnetometer, so H @ p is a single
    // block row of p's 3x3 blocks:
    //   HP_j = h0 @ p[0:3, 3j:3j+3] + H2 @ p[6:9, 3j:3j+3]
    float hp0[9], hp1[9], hp2[9];
    float* hp[3] = {hp0, hp1, hp2};
    float pBlock[9], tmp[9];
    for (int j = 0; j < 3; ++j) {
        getBlock3x3(p, 0, j, pBlock);
        math->matrixMult(h0, 3, 3, pBlock, 3, hp[j]);
        if (observesAccelBias) {
            getBlock3x3(p, 2, j, pBlock);
            math->matrixAdd(hp[j], pBlock, hp[j], 3, 3);
        }
    }

    // 1. Mahalanobis Gating: s = H @ p @ H^t + R = HP_0 @ h0^t + HP_2 @ H2^t + R
    float h0T[9], s[9];
    math->matrixTranspose(h0, 3, 3, h0T);
    math->matrixMult(hp0, 3, 3, h0T, 3, s);
    if (observesAccelBias) {
        math->matrixAdd(s, hp2, s, 3, 3);
    }
    math->matrixAdd(s, R, s, 3, 3);

    float sInv[9]; // 3x3
    if (!math->matrixInverse(s, 3, sInv)) return; // Failsafe against singularity

    float yTSinv[3]; // 1x3
    math->matrixMult(y, 1, 3, sInv, 3, yTSinv); // y_T is identical memory layout as y

    float mahalanobisDist = 0;
    math->matrixMult(yTSinv, 1, 3, y, 1, &mahalanobisDist);

    if (mahalanobisDist > gateThreshold) return;

    // 2. Kalman Update: k = p @ H^t @ sInv; p is symmetric, so p @ H^t = (H @ p)^t
    // and each 3x3 block row of k is K_i = HP_i^t @ sInv
    float k0[9], k1[9], k2[9];
    float* k[3] = {k0, k1, k2};
    for (int i = 0; i < 3; ++i) {
        math->matrixTranspose(hp[i], 3, 3, tmp);
        math->matrixMult(tmp, 3, 3, sInv, 3, k[i]);
    }

    // errorState = k @ y
    float errorState[9]; // 9x1
    for (int i = 0; i < 3; ++i) {
        math->matrixMult(k[i], 3, 3, y, 1, &errorState[i*3]);
    }

    // p = (I - k @ H) @ p = p - k @ (H @ p), one 3x3 block at a time:
    // block (i, j) of k @ (H @ p) is K_i @ HP_j
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            math->matrixMult(k[i], 3, 3, hp[j], 3, tmp);
            getBlock3x3(p, i, j, pBlock);
            math->matrixSub(pBlock, tmp, pBlock, 3, 3);
            setBlock3x3(p, i, j, pBlock);
        }
    }

    // 3. Update States & Biases
    nom.correctState(&errorState[0]);
    meas.updateBiases(&errorState[3], &errorState[6], nullptr);

    // 4. Reset Error State Jacobian: p = J @ p @ J^t, with J = I except
    // J[0:3, 0:3] = I - 0.5 * skew(err), so only p's first block row/column changes:
    //   p00' = j00 @ p00 @ j00^t
    //   p0j' = j00 @ p0j and Pj0' = Pj0 @ j00^t, for j = 1, 2
    float skewErr[9], j00[9];
    math->skewSymmetric(&errorState[0], skewErr);
    math->matrixScale(skewErr, -0.5f, j00, 3, 3);
    j00[0] += 1.0f; j00[4] += 1.0f; j00[8] += 1.0f;

    float j00T[9];
    math->matrixTranspose(j00, 3, 3, j00T);

    getBlock3x3(p, 0, 0, pBlock);
    math->matrixMult(j00, 3, 3, pBlock, 3, tmp);
    math->matrixMult(tmp, 3, 3, j00T, 3, pBlock);
    setBlock3x3(p, 0, 0, pBlock);

    for (int j = 1; j < 3; ++j) {
        getBlock3x3(p, 0, j, pBlock);
        math->matrixMult(j00, 3, 3, pBlock, 3, tmp);
        setBlock3x3(p, 0, j, tmp);

        getBlock3x3(p, j, 0, pBlock);
        math->matrixMult(pBlock, 3, 3, j00T, 3, tmp);
        setBlock3x3(p, j, 0, tmp);
    }

    math->ensureSymmetric(p, 9);
}

Attitude_t AhrsEsmEkf::getAttitudeRadians() const {
    float eAHRSEKF;
    math->quatToEuler(nom.quaternionNew, eulerTmp);

    Attitude_t att;
    att.roll  = eulerTmp[0];
    att.pitch = eulerTmp[1];
    att.yaw   = eulerTmp[2];

    return att;
}
