#include "mlx90393.hpp"

#include <cmath>

static constexpr uint16_t GAIN_MASK = 0x0070;
static constexpr uint8_t GAIN_SHIFT = 4;
static constexpr uint16_t HALLCONF_MASK = 0x000F;
static constexpr uint16_t TCMP_EN_MASK = 0x0400;
static constexpr uint8_t TCMP_EN_SHIFT = 10;

static constexpr uint16_t X_RES_MASK = 0x0060;
static constexpr uint16_t Y_RES_MASK = 0x0180;
static constexpr uint16_t Z_RES_MASK = 0x0600;
static constexpr uint8_t X_RES_SHIFT = 5;
static constexpr uint8_t Y_RES_SHIFT = 7;
static constexpr uint8_t Z_RES_SHIFT = 9;

static constexpr uint16_t FILTER_MASK = 0x001C;
static constexpr uint8_t FILTER_SHIFT = 2;
static constexpr uint16_t OSR_MASK = 0x0003;
static constexpr uint8_t OSR_SHIFT = 0;

static constexpr int32_t RES17_ZERO_OFFSET = 0x8000;
static constexpr int32_t RES18_ZERO_OFFSET = 0x4000;

static constexpr uint8_t HALLCONF_0XC = 0x0C;

// uT/LSB by [gain][res][XY|Z], datasheet 16.2.4
static constexpr float SENS_LOOKUP_0XC[8][4][2] = {
     {{0.751f, 1.210f}, {1.502f, 2.420f}, {3.004f, 4.840f}, {6.009f, 9.680f}},
     {{0.601f, 0.968f}, {1.202f, 1.936f}, {2.403f, 3.872f}, {4.840f, 7.744f}},
     {{0.451f, 0.726f}, {0.901f, 1.452f}, {1.803f, 2.904f}, {3.605f, 5.808f}},
     {{0.376f, 0.605f}, {0.751f, 1.210f}, {1.502f, 2.420f}, {3.004f, 4.840f}},
     {{0.300f, 0.484f}, {0.601f, 0.968f}, {1.202f, 1.936f}, {2.403f, 3.872f}},
     {{0.250f, 0.403f}, {0.501f, 0.807f}, {1.001f, 1.613f}, {2.003f, 3.227f}},
     {{0.200f, 0.323f}, {0.401f, 0.645f}, {0.801f, 1.291f}, {1.602f, 2.581f}},
     {{0.150f, 0.242f}, {0.300f, 0.484f}, {0.601f, 0.968f}, {1.202f, 1.936f}},
};

// ms by [filter][osr], datasheet 16.2.5
static constexpr float TCONV_MS[8][4] = {
     {1.27f, 1.84f, 3.00f, 5.30f},
     {1.46f, 2.23f, 3.76f, 6.84f},
     {1.84f, 3.00f, 5.30f, 9.91f},
     {2.61f, 4.53f, 8.37f, 16.05f},
     {4.15f, 7.60f, 14.52f, 28.34f},
     {7.22f, 13.75f, 26.80f, 52.92f},
     {13.36f, 26.04f, 51.38f, 102.07f},
     {25.65f, 50.61f, 100.53f, 200.37f},
};

static constexpr uint8_t CONFIG_FILTER = 0x05;
static constexpr uint8_t CONFIG_OSR = 0x03;
static constexpr uint8_t CONFIG_X_RES = MLX90393_RES_16;
static constexpr uint8_t CONFIG_Y_RES = MLX90393_RES_16;
static constexpr uint8_t CONFIG_Z_RES = MLX90393_RES_15;

static constexpr MagCalConstants_t DEFAULT_CAL_CONSTANTS = {

    {-9.946797043f, -25.11398367f, 3.1675524f},

    {1.00266659f, 0.0f, 0.0f,
     0.0f, 1.35954797f, 0.0f,
     0.0f, 0.0f, 0.78919065f},

    53.5f, // ideal
};

Magnetometer::Magnetometer(I2C_HandleTypeDef *hi2c, GPIO_TypeDef *csPort, uint16_t csPin, uint8_t address)
    : hi2c(hi2c),
      csPort(csPort),
      csPin(csPin),
      halAddress(static_cast<uint16_t>(address) << 1),
      gain(MLX90393_GAIN_1X),
      xRes(CONFIG_X_RES),
      yRes(CONFIG_Y_RES),
      zRes(CONFIG_Z_RES),
      hallconf(HALLCONF_0XC),
      tcmpEn(0),
      filter(CONFIG_FILTER),
      osr(CONFIG_OSR),
      status(0),
      regValue(0),
      state(IDLE),
      sampleReady(false),
      conversionDoneMs(0),
      transferStartMs(0),
      txByte(0),
      rxBuffer{0},
      magData{},
      calConstants(DEFAULT_CAL_CONSTANTS),
      calState(MAG_CAL_STATE_IDLE),
      calError(MAG_CAL_ERR_NONE),
      calStartMs(0),
      calSampleCount(0),
      calNormalMatrix{0},
      calNormalVector{0},
      calAxisMin{0},
      calAxisMax{0} {}

bool Magnetometer::init() {

    if (csPort != nullptr) {
        HAL_GPIO_WritePin(csPort, csPin, GPIO_PIN_SET);
        HAL_Delay(2);
    }

    if (HAL_I2C_IsDeviceReady(hi2c, halAddress, 2, INIT_TIMEOUT_MS) != HAL_OK) {
        return false;
    }

    sendCommand(MLX90393_CMD_EXIT);
    HAL_Delay(1);

    if (!sendCommand(MLX90393_CMD_RESET)) {
        return false;
    }
    HAL_Delay(2);

    if (!refreshConfigCache()) {
        return false;
    }

    if (!setFilter(CONFIG_FILTER)) {
        return false;
    }
    if (!setOversampling(CONFIG_OSR)) {
        return false;
    }
    if (!setResolution(CONFIG_X_RES, CONFIG_Y_RES, CONFIG_Z_RES)) {
        return false;
    }

    if (!refreshConfigCache()) {
        return false;
    }

    state = IDLE;
    sampleReady = false;
    magData = {};

    return true;
}

bool Magnetometer::transceive(uint8_t *txData, uint16_t txSize, uint8_t *rxData, uint16_t rxSize) {
    if (HAL_I2C_Master_Transmit(hi2c, halAddress, txData, txSize, INIT_TIMEOUT_MS) != HAL_OK) {
        return false;
    }
    return HAL_I2C_Master_Receive(hi2c, halAddress, rxData, rxSize, INIT_TIMEOUT_MS) == HAL_OK;
}

bool Magnetometer::sendCommand(uint8_t command) {
    uint8_t tx = command;
    uint8_t rx = 0;
    if (!transceive(&tx, 1, &rx, 1)) {
        return false;
    }
    status = rx;
    return true;
}

bool Magnetometer::writeRegister(uint8_t reg, uint16_t value) {
    uint8_t tx[4] = {
        MLX90393_CMD_WRITE_REGISTER,
        static_cast<uint8_t>(value >> 8),
        static_cast<uint8_t>(value & 0xFF),
        static_cast<uint8_t>(reg << 2),
    };
    uint8_t rx = 0;
    if (!transceive(tx, 4, &rx, 1)) {
        return false;
    }
    status = rx;
    return true;
}

bool Magnetometer::readRegister(uint8_t reg) {
    uint8_t tx[2] = {MLX90393_CMD_READ_REGISTER, static_cast<uint8_t>(reg << 2)};
    uint8_t rx[3] = {0};
    if (!transceive(tx, 2, rx, 3)) {
        return false;
    }
    status = rx[0];
    regValue = static_cast<uint16_t>(rx[1] << 8 | rx[2]);
    return true;
}

bool Magnetometer::refreshConfigCache() {
    if (!readRegister(MLX90393_CONF1)) {
        return false;
    }
    gain = (regValue & GAIN_MASK) >> GAIN_SHIFT;
    hallconf = regValue & HALLCONF_MASK;
    tcmpEn = (regValue & TCMP_EN_MASK) >> TCMP_EN_SHIFT;

    if (!readRegister(MLX90393_CONF3)) {
        return false;
    }
    xRes = (regValue & X_RES_MASK) >> X_RES_SHIFT;
    yRes = (regValue & Y_RES_MASK) >> Y_RES_SHIFT;
    zRes = (regValue & Z_RES_MASK) >> Z_RES_SHIFT;
    filter = (regValue & FILTER_MASK) >> FILTER_SHIFT;
    osr = (regValue & OSR_MASK) >> OSR_SHIFT;

    return true;
}

bool Magnetometer::setGain(uint8_t newGain) {
    if (!readRegister(MLX90393_CONF1)) {
        return false;
    }
    const uint16_t VALUE = (regValue & ~GAIN_MASK) | (newGain << GAIN_SHIFT);
    if (!writeRegister(MLX90393_CONF1, VALUE)) {
        return false;
    }
    gain = newGain;
    return true;
}

bool Magnetometer::setResolution(uint8_t newXRes, uint8_t newYRes, uint8_t newZRes) {

    if (tcmpEn == 1 &&
        (newXRes >= MLX90393_RES_17 || newYRes >= MLX90393_RES_17 || newZRes >= MLX90393_RES_17)) {
        return false;
    }

    if (!readRegister(MLX90393_CONF3)) {
        return false;
    }
    uint16_t value = regValue;
    value = (value & ~X_RES_MASK) | (newXRes << X_RES_SHIFT);
    value = (value & ~Y_RES_MASK) | (newYRes << Y_RES_SHIFT);
    value = (value & ~Z_RES_MASK) | (newZRes << Z_RES_SHIFT);

    if (!writeRegister(MLX90393_CONF3, value)) {
        return false;
    }
    xRes = newXRes;
    yRes = newYRes;
    zRes = newZRes;
    return true;
}

bool Magnetometer::setFilter(uint8_t newFilter) {

    if (hallconf == HALLCONF_0XC) {
        if (osr == 0x00 && (newFilter == 0 || newFilter == 1)) {
            return false;
        }
        if (osr == 0x01 && newFilter == 0) {
            return false;
        }
    }

    if (!readRegister(MLX90393_CONF3)) {
        return false;
    }
    const uint16_t VALUE = (regValue & ~FILTER_MASK) | (newFilter << FILTER_SHIFT);
    if (!writeRegister(MLX90393_CONF3, VALUE)) {
        return false;
    }
    filter = newFilter;
    return true;
}

bool Magnetometer::setOversampling(uint8_t newOsr) {

    if (hallconf == HALLCONF_0XC) {
        if (filter == 0x00 && (newOsr == 0 || newOsr == 1)) {
            return false;
        }
        if (filter == 0x01 && newOsr == 0) {
            return false;
        }
    }

    if (!readRegister(MLX90393_CONF3)) {
        return false;
    }
    const uint16_t VALUE = (regValue & ~OSR_MASK) | (newOsr << OSR_SHIFT);
    if (!writeRegister(MLX90393_CONF3, VALUE)) {
        return false;
    }
    osr = newOsr;
    return true;
}

MagData_t Magnetometer::readData() {

    if (sampleReady) {
        sampleReady = false;
        decode();
    } else {
        magData.isNew = false;
    }

    const uint32_t NOW = HAL_GetTick();

    switch (state) {
        case IDLE:
            startMeasurement();
            break;

        case WAIT_CONV:
            if (static_cast<int32_t>(NOW - conversionDoneMs) >= 0) {
                readMeasurement();
            }
            break;

        default:

            if (NOW - transferStartMs > TRANSFER_TIMEOUT_MS &&
                hi2c->State == HAL_I2C_STATE_READY) {
                state = IDLE;
            }
            break;
    }

    return magData;
}

void Magnetometer::startMeasurement() {
    txByte = MLX90393_CMD_START_MEASUREMENT | ZYXT;
    transferStartMs = HAL_GetTick();
    state = SM_TX;

    if (HAL_I2C_Master_Transmit_IT(hi2c, halAddress, &txByte, 1) != HAL_OK) {
        state = IDLE;
    }
}

void Magnetometer::readMeasurement() {
    txByte = MLX90393_CMD_READ_MEASUREMENT | ZYXT;
    transferStartMs = HAL_GetTick();
    state = RM_TX;

    if (HAL_I2C_Master_Transmit_IT(hi2c, halAddress, &txByte, 1) != HAL_OK) {
        state = IDLE;
    }
}

void Magnetometer::masterTxCpltCallback() {
    switch (state) {
        case SM_TX:
            state = SM_RX;

            if (HAL_I2C_Master_Receive_IT(hi2c, halAddress, &status, 1) != HAL_OK) {
                state = IDLE;
            }
            break;

        case RM_TX:
            state = RM_RX;
            if (HAL_I2C_Master_Receive_IT(hi2c, halAddress, rxBuffer, MEASUREMENT_BYTES) != HAL_OK) {
                state = IDLE;
            }
            break;

        default:
            break;
    }
}

void Magnetometer::masterRxCpltCallback() {
    switch (state) {
        case SM_RX:
            if (status & MLX90393_STATUS_ERROR) {

                state = IDLE;
                break;
            }
            conversionDoneMs = HAL_GetTick() + conversionTimeMs();
            state = WAIT_CONV;
            break;

        case RM_RX:
            state = IDLE;
            sampleReady = true;
            break;

        default:
            state = IDLE;
            break;
    }
}

void Magnetometer::errorCallback() {

    state = IDLE;
}

uint32_t Magnetometer::conversionTimeMs() const {
    const float TCONV = TCONV_MS[filter & 0x07][osr & 0x03];
    return static_cast<uint32_t>(TCONV) + 1 + CONVERSION_MARGIN_MS;
}

void Magnetometer::decode() {
    const uint8_t SAMPLE_STATUS = rxBuffer[0];

    if (SAMPLE_STATUS & MLX90393_STATUS_ERROR) {
        magData.isNew = false;
        return;
    }

    const uint8_t *cursor = rxBuffer + 1;

    int16_t raw[3] = {0, 0, 0};

    if (ZYXT & MLX90393_AXIS_T) {
        cursor += 2;
    }
    for (uint8_t i = 0; i < 3; i++) {
        raw[i] = static_cast<int16_t>((cursor[0] << 8) | cursor[1]);
        cursor += 2;
    }

    int32_t counts[3] = {raw[0], raw[1], raw[2]};

    if (tcmpEn == 0) {
        const uint8_t RES[3] = {xRes, yRes, zRes};
        for (uint8_t i = 0; i < 3; i++) {
            if (RES[i] == MLX90393_RES_17) {
                counts[i] = static_cast<int32_t>(static_cast<uint16_t>(raw[i])) - RES17_ZERO_OFFSET;
            } else if (RES[i] == MLX90393_RES_18) {
                counts[i] = static_cast<int32_t>(static_cast<uint16_t>(raw[i])) - RES18_ZERO_OFFSET;
            }
        }
    }

    const uint8_t G = gain & 0x07;
    float field[3] = {
        static_cast<float>(counts[0]) * SENS_LOOKUP_0XC[G][xRes & 0x03][0],
        static_cast<float>(counts[1]) * SENS_LOOKUP_0XC[G][yRes & 0x03][0],
        static_cast<float>(counts[2]) * SENS_LOOKUP_0XC[G][zRes & 0x03][1],
    };

    const uint32_t NOW = HAL_GetTick();

    calAddSample(field, NOW);

    calApply(field);

    magData.x = field[0];
    magData.y = field[1];
    magData.z = field[2];
    magData.rawX = raw[0];
    magData.rawY = raw[1];
    magData.rawZ = raw[2];
    magData.timestamp = NOW;
    magData.isNew = true;
}

I2C_HandleTypeDef *Magnetometer::getI2C() {
    return hi2c;
}

void Magnetometer::startCalibration() {
    if (calState == MAG_CAL_STATE_COLLECTING) {
        return;
    }

    calResetAccumulators();
    calStartMs = HAL_GetTick();
    calError = MAG_CAL_ERR_NONE;
    calState = MAG_CAL_STATE_COLLECTING;
}

void Magnetometer::cancelCalibration() {
    if (calState != MAG_CAL_STATE_COLLECTING) {
        return;
    }
    calResetAccumulators();
    calState = MAG_CAL_STATE_IDLE;
}

void Magnetometer::resetCalibrationToDefaults() {
    calConstants = DEFAULT_CAL_CONSTANTS;
    calError = MAG_CAL_ERR_NONE;
    calState = MAG_CAL_STATE_IDLE;
}

MagCalStatus_t Magnetometer::getCalibrationStatus() {
    MagCalStatus_t out = {};
    out.state = calState;
    out.error = calError;
    out.sampleCount = calSampleCount;

    if (calState != MAG_CAL_STATE_COLLECTING) {
        out.progressPercent = (calState == MAG_CAL_STATE_SUCCESS) ? 100 : 0;
        return out;
    }

    const uint32_t ELAPSED = HAL_GetTick() - calStartMs;
    out.progressPercent = (ELAPSED >= CAL_COLLECT_DURATION_MS)
                              ? 100
                              : static_cast<uint8_t>((ELAPSED * 100U) / CAL_COLLECT_DURATION_MS);
    return out;
}

const MagCalConstants_t &Magnetometer::getCalibrationConstants() const {
    return calConstants;
}

void Magnetometer::calApply(float field[3]) const {
    const float CENTERED[3] = {
        field[0] - calConstants.hardIron[0],
        field[1] - calConstants.hardIron[1],
        field[2] - calConstants.hardIron[2],
    };

    for (uint8_t row = 0; row < 3; row++) {
        field[row] = calConstants.softIron[row * 3 + 0] * CENTERED[0] +
                     calConstants.softIron[row * 3 + 1] * CENTERED[1] +
                     calConstants.softIron[row * 3 + 2] * CENTERED[2];
    }
}

void Magnetometer::calResetAccumulators() {
    for (uint8_t i = 0; i < 16; i++) {
        calNormalMatrix[i] = 0.0f;
    }
    for (uint8_t i = 0; i < 4; i++) {
        calNormalVector[i] = 0.0f;
    }
    for (uint8_t i = 0; i < 3; i++) {
        calAxisMin[i] = 1e9f;
        calAxisMax[i] = -1e9f;
    }
    calSampleCount = 0;
}

void Magnetometer::calAddSample(const float field[3], uint32_t nowMs) {
    if (calState != MAG_CAL_STATE_COLLECTING) {
        return;
    }

    const float X = field[0];
    const float Y = field[1];
    const float Z = field[2];
    const float R2 = X * X + Y * Y + Z * Z;

    calNormalMatrix[0] += X * X;
    calNormalMatrix[1] += X * Y;
    calNormalMatrix[2] += X * Z;
    calNormalMatrix[3] += X;

    calNormalMatrix[4] += X * Y;
    calNormalMatrix[5] += Y * Y;
    calNormalMatrix[6] += Y * Z;
    calNormalMatrix[7] += Y;

    calNormalMatrix[8] += X * Z;
    calNormalMatrix[9] += Y * Z;
    calNormalMatrix[10] += Z * Z;
    calNormalMatrix[11] += Z;

    calNormalMatrix[12] += X;
    calNormalMatrix[13] += Y;
    calNormalMatrix[14] += Z;
    calNormalMatrix[15] += 1.0f;

    calNormalVector[0] += X * R2;
    calNormalVector[1] += Y * R2;
    calNormalVector[2] += Z * R2;
    calNormalVector[3] += R2;

    if (X < calAxisMin[0]) calAxisMin[0] = X;
    if (X > calAxisMax[0]) calAxisMax[0] = X;
    if (Y < calAxisMin[1]) calAxisMin[1] = Y;
    if (Y > calAxisMax[1]) calAxisMax[1] = Y;
    if (Z < calAxisMin[2]) calAxisMin[2] = Z;
    if (Z > calAxisMax[2]) calAxisMax[2] = Z;

    calSampleCount++;

    if (nowMs - calStartMs < CAL_COLLECT_DURATION_MS) {
        return;
    }

    calState = calSolve() ? MAG_CAL_STATE_SUCCESS : MAG_CAL_STATE_FAILED;
}

bool Magnetometer::invert4x4(const float src[16], float dst[16]) {

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

bool Magnetometer::calSolve() {
    if (calSampleCount < CAL_MIN_SAMPLES) {
        calError = MAG_CAL_ERR_TOO_FEW_SAMPLES;
        return false;
    }

    float radius[3];
    for (uint8_t i = 0; i < 3; i++) {
        radius[i] = 0.5f * (calAxisMax[i] - calAxisMin[i]);
    }

    float radiusMax = radius[0];
    for (uint8_t i = 1; i < 3; i++) {
        if (radius[i] > radiusMax) radiusMax = radius[i];
    }

    if (radiusMax < CAL_MIN_AXIS_SPAN_UT ||
        radius[0] < CAL_MIN_SPAN_RATIO * radiusMax ||
        radius[1] < CAL_MIN_SPAN_RATIO * radiusMax ||
        radius[2] < CAL_MIN_SPAN_RATIO * radiusMax) {
        calError = MAG_CAL_ERR_POOR_COVERAGE;
        return false;
    }

    float inverse[16];
    if (!invert4x4(calNormalMatrix, inverse)) {
        calError = MAG_CAL_ERR_SINGULAR;
        return false;
    }

    float beta[4] = {0.0f, 0.0f, 0.0f, 0.0f};
    for (uint8_t row = 0; row < 4; row++) {
        for (uint8_t col = 0; col < 4; col++) {
            beta[row] += inverse[row * 4 + col] * calNormalVector[col];
        }
    }

    // beta = [2b, |B|^2 - |b|^2], NXP AN4246
    const float BX = 0.5f * beta[0];
    const float BY = 0.5f * beta[1];
    const float BZ = 0.5f * beta[2];
    const float FIELD_SQUARED = beta[3] + BX * BX + BY * BY + BZ * BZ;

    if (FIELD_SQUARED <= 0.0f) {
        calError = MAG_CAL_ERR_IMPLAUSIBLE_FIELD;
        return false;
    }

    const float FIELD = sqrtf(FIELD_SQUARED);
    if (FIELD < CAL_MIN_FIELD_UT || FIELD > CAL_MAX_FIELD_UT) {
        calError = MAG_CAL_ERR_IMPLAUSIBLE_FIELD;
        return false;
    }

    calConstants.hardIron[0] = BX;
    calConstants.hardIron[1] = BY;
    calConstants.hardIron[2] = BZ;
    calConstants.fieldStrength = FIELD;

    const float RADIUS_MEAN = (radius[0] + radius[1] + radius[2]) / 3.0f;
    for (uint8_t i = 0; i < 9; i++) {
        calConstants.softIron[i] = 0.0f;
    }
    calConstants.softIron[0] = RADIUS_MEAN / radius[0];
    calConstants.softIron[4] = RADIUS_MEAN / radius[1];
    calConstants.softIron[8] = RADIUS_MEAN / radius[2];

    calError = MAG_CAL_ERR_NONE;
    return true;
}
