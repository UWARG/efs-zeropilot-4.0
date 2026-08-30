#include "mag_cal.hpp"

#include <cmath>

/*
 * Identity by default: MagCal is sensor agnostic, so it cannot assume the
 * distortion of any particular board. A fit measured on one airframe would
 * corrupt every other sensor, including the SITL model, which already reports
 * an undistorted field. Real constants come from startCalibration() or from
 * setConstants() once they are persisted in NVM / ZP params.
 *
 * For reference, the fit measured on the ZP4.0 MLX90393 bench unit was
 *   hardIron {-9.946797043, -25.11398367, 3.1675524} uT
 *   softIron diag {1.00266659, 1.35954797, 0.78919065}
 *   fieldStrength 53.5 uT
 */
static constexpr MagCalConstants_t DEFAULT_CAL_CONSTANTS = {
    {0.0f, 0.0f, 0.0f},

    {1.0f, 0.0f, 0.0f,
     0.0f, 1.0f, 0.0f,
     0.0f, 0.0f, 1.0f},

    50.0f, // nominal Earth field, which ranges roughly 25-65 uT
};

MagCal::MagCal(IMagnetometer *magDriver)
    : magDriver(magDriver),
      constants(DEFAULT_CAL_CONSTANTS),
      state(MAG_CAL_STATE_IDLE),
      error(MAG_CAL_ERR_NONE),
      startMs(0),
      lastSampleMs(0),
      haveStartMs(false),
      sampleCount(0),
      normalMatrix{0},
      normalVector{0},
      axisMin{0},
      axisMax{0} {}

MagData_t MagCal::update() {
    MagData_t data = magDriver->readData();

    if (!data.isNew) {
        return data;
    }

    float field[3] = {data.x, data.y, data.z};

    // The fit models the distortion of the uncorrected field, so accumulate
    // before applying the correction.
    if (state == MAG_CAL_STATE_COLLECTING) {
        addSample(field, data.timestamp);
    }

    applyCorrection(field);

    data.x = field[0];
    data.y = field[1];
    data.z = field[2];
    return data;
}

void MagCal::startCalibration() {
    if (state == MAG_CAL_STATE_COLLECTING) {
        return;
    }

    resetAccumulators();
    error = MAG_CAL_ERR_NONE;
    state = MAG_CAL_STATE_COLLECTING;
}

void MagCal::cancelCalibration() {
    if (state != MAG_CAL_STATE_COLLECTING) {
        return;
    }

    resetAccumulators();
    state = MAG_CAL_STATE_IDLE;
}

MagCalStatus_t MagCal::getStatus() const {
    MagCalStatus_t out = {};
    out.state = state;
    out.error = error;
    out.sampleCount = sampleCount;

    if (state != MAG_CAL_STATE_COLLECTING) {
        out.progressPercent = (state == MAG_CAL_STATE_SUCCESS) ? 100 : 0;
        return out;
    }

    // The window opens on the first sample, so there is no progress to report
    // until one has landed.
    if (!haveStartMs) {
        out.progressPercent = 0;
        return out;
    }

    const uint32_t ELAPSED = lastSampleMs - startMs;
    out.progressPercent = (ELAPSED >= COLLECT_DURATION_MS)
                              ? 100
                              : static_cast<uint8_t>((ELAPSED * 100U) / COLLECT_DURATION_MS);
    return out;
}

const MagCalConstants_t &MagCal::getConstants() const {
    return constants;
}

void MagCal::setConstants(const MagCalConstants_t &newConstants) {
    constants = newConstants;
    error = MAG_CAL_ERR_NONE;
    state = MAG_CAL_STATE_IDLE;
}

void MagCal::resetToDefaults() {
    setConstants(DEFAULT_CAL_CONSTANTS);
}

void MagCal::applyCorrection(float field[3]) const {
    const float CENTERED[3] = {
        field[0] - constants.hardIron[0],
        field[1] - constants.hardIron[1],
        field[2] - constants.hardIron[2],
    };

    for (uint8_t row = 0; row < 3; row++) {
        field[row] = constants.softIron[row * 3 + 0] * CENTERED[0] +
                     constants.softIron[row * 3 + 1] * CENTERED[1] +
                     constants.softIron[row * 3 + 2] * CENTERED[2];
    }
}

void MagCal::resetAccumulators() {
    for (uint8_t i = 0; i < 16; i++) {
        normalMatrix[i] = 0.0f;
    }
    for (uint8_t i = 0; i < 4; i++) {
        normalVector[i] = 0.0f;
    }
    for (uint8_t i = 0; i < 3; i++) {
        axisMin[i] = 1e9f;
        axisMax[i] = -1e9f;
    }
    sampleCount = 0;
    startMs = 0;
    lastSampleMs = 0;
    haveStartMs = false;
}

void MagCal::addSample(const float field[3], uint32_t nowMs) {
    if (state != MAG_CAL_STATE_COLLECTING) {
        return;
    }

    // The window is measured in sensor time rather than against a clock this
    // class does not own, so it opens on the first sample.
    if (!haveStartMs) {
        startMs = nowMs;
        haveStartMs = true;
    }
    lastSampleMs = nowMs;

    const float X = field[0];
    const float Y = field[1];
    const float Z = field[2];
    const float R2 = X * X + Y * Y + Z * Z;

    normalMatrix[0] += X * X;
    normalMatrix[1] += X * Y;
    normalMatrix[2] += X * Z;
    normalMatrix[3] += X;

    normalMatrix[4] += X * Y;
    normalMatrix[5] += Y * Y;
    normalMatrix[6] += Y * Z;
    normalMatrix[7] += Y;

    normalMatrix[8] += X * Z;
    normalMatrix[9] += Y * Z;
    normalMatrix[10] += Z * Z;
    normalMatrix[11] += Z;

    normalMatrix[12] += X;
    normalMatrix[13] += Y;
    normalMatrix[14] += Z;
    normalMatrix[15] += 1.0f;

    normalVector[0] += X * R2;
    normalVector[1] += Y * R2;
    normalVector[2] += Z * R2;
    normalVector[3] += R2;

    if (X < axisMin[0]) axisMin[0] = X;
    if (X > axisMax[0]) axisMax[0] = X;
    if (Y < axisMin[1]) axisMin[1] = Y;
    if (Y > axisMax[1]) axisMax[1] = Y;
    if (Z < axisMin[2]) axisMin[2] = Z;
    if (Z > axisMax[2]) axisMax[2] = Z;

    sampleCount++;

    if (nowMs - startMs < COLLECT_DURATION_MS) {
        return;
    }

    state = solve() ? MAG_CAL_STATE_SUCCESS : MAG_CAL_STATE_FAILED;
}

bool MagCal::invert4x4(const float src[16], float dst[16]) {

    double aug[4][8] = {};
    for (uint8_t i = 0; i < 4; i++) {
        for (uint8_t j = 0; j < 4; j++) {
            aug[i][j] = src[i * 4 + j];
        }
        aug[i][4 + i] = 1.0;
    }

    for (uint8_t col = 0; col < 4; col++) {
        uint8_t pivot = col;
        for (uint8_t row = col + 1; row < 4; row++) {
            if (std::fabs(aug[row][col]) > std::fabs(aug[pivot][col])) {
                pivot = row;
            }
        }
        if (std::fabs(aug[pivot][col]) < 1e-12) {
            return false;
        }
        if (pivot != col) {
            for (uint8_t j = 0; j < 8; j++) {
                const double TMP = aug[col][j];
                aug[col][j] = aug[pivot][j];
                aug[pivot][j] = TMP;
            }
        }

        const double DIAG = aug[col][col];
        for (uint8_t j = 0; j < 8; j++) {
            aug[col][j] /= DIAG;
        }
        for (uint8_t row = 0; row < 4; row++) {
            if (row == col) continue;
            const double FACTOR = aug[row][col];
            if (FACTOR == 0.0) continue;
            for (uint8_t j = 0; j < 8; j++) {
                aug[row][j] -= FACTOR * aug[col][j];
            }
        }
    }

    for (uint8_t i = 0; i < 4; i++) {
        for (uint8_t j = 0; j < 4; j++) {
            dst[i * 4 + j] = static_cast<float>(aug[i][4 + j]);
        }
    }
    return true;
}

bool MagCal::solve() {
    if (sampleCount < MIN_SAMPLES) {
        error = MAG_CAL_ERR_TOO_FEW_SAMPLES;
        return false;
    }

    float radius[3];
    for (uint8_t i = 0; i < 3; i++) {
        radius[i] = 0.5f * (axisMax[i] - axisMin[i]);
    }

    float radiusMax = radius[0];
    for (uint8_t i = 1; i < 3; i++) {
        if (radius[i] > radiusMax) radiusMax = radius[i];
    }

    if (radiusMax < MIN_AXIS_SPAN_UT ||
        radius[0] < MIN_SPAN_RATIO * radiusMax ||
        radius[1] < MIN_SPAN_RATIO * radiusMax ||
        radius[2] < MIN_SPAN_RATIO * radiusMax) {
        error = MAG_CAL_ERR_POOR_COVERAGE;
        return false;
    }

    float inverse[16];
    if (!invert4x4(normalMatrix, inverse)) {
        error = MAG_CAL_ERR_SINGULAR;
        return false;
    }

    float beta[4] = {0.0f, 0.0f, 0.0f, 0.0f};
    for (uint8_t row = 0; row < 4; row++) {
        for (uint8_t col = 0; col < 4; col++) {
            beta[row] += inverse[row * 4 + col] * normalVector[col];
        }
    }

    // beta = [2b, |B|^2 - |b|^2], NXP AN4246
    const float BX = 0.5f * beta[0];
    const float BY = 0.5f * beta[1];
    const float BZ = 0.5f * beta[2];
    const float FIELD_SQUARED = beta[3] + BX * BX + BY * BY + BZ * BZ;

    if (FIELD_SQUARED <= 0.0f) {
        error = MAG_CAL_ERR_IMPLAUSIBLE_FIELD;
        return false;
    }

    const float FIELD = sqrtf(FIELD_SQUARED);
    if (FIELD < MIN_FIELD_UT || FIELD > MAX_FIELD_UT) {
        error = MAG_CAL_ERR_IMPLAUSIBLE_FIELD;
        return false;
    }

    constants.hardIron[0] = BX;
    constants.hardIron[1] = BY;
    constants.hardIron[2] = BZ;
    constants.fieldStrength = FIELD;

    const float RADIUS_MEAN = (radius[0] + radius[1] + radius[2]) / 3.0f;
    for (uint8_t i = 0; i < 9; i++) {
        constants.softIron[i] = 0.0f;
    }
    constants.softIron[0] = RADIUS_MEAN / radius[0];
    constants.softIron[4] = RADIUS_MEAN / radius[1];
    constants.softIron[8] = RADIUS_MEAN / radius[2];

    error = MAG_CAL_ERR_NONE;
    return true;
}
