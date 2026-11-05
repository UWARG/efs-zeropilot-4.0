#include "mlx90393.hpp"
#include "utils.hpp"

// Register fields go over the wire as raw bytes
template <typename EnumT>
static constexpr uint8_t toByte(EnumT value) {
    return static_cast<uint8_t>(value);
}

enum class MLX90393_AXIS : uint8_t {
    T = 0x01,
    X = 0x02,
    Y = 0x04,
    Z = 0x08,
};

enum class MLX90393_REG : uint8_t {
    CONF1 = 0x00,
    CONF2 = 0x01,
    CONF3 = 0x02,
    CONF4 = 0x03,
};

enum class MLX90393_CMD : uint8_t {
    NOP               = 0x00,
    EXIT              = 0x80,
    START_BURST       = 0x10,
    WAKE_ON_CHANGE    = 0x20,
    START_MEASUREMENT = 0x30,
    READ_MEASUREMENT  = 0x40,
    READ_REGISTER     = 0x50,
    WRITE_REGISTER    = 0x60,
    MEMORY_RECALL     = 0xD0,
    MEMORY_STORE      = 0xE0,
    RESET             = 0xF0,
};

enum class MLX90393_STATUS : uint8_t {
    BURST_MODE     = 0x80,
    WAKE_ON_CHANGE = 0x40,
    POLLING_MODE   = 0x20,
    ERROR          = 0x10,
    EEC            = 0x08,
    RESET          = 0x04,
};

enum class MLX90393_GAIN : uint8_t {
    X5    = 0x00,
    X4    = 0x01,
    X3    = 0x02,
    X2_5  = 0x03,
    X2    = 0x04,
    X1_67 = 0x05,
    X1_33 = 0x06,
    X1    = 0x07,
};

enum class MLX90393_RES : uint8_t {
    BITS_15 = 0x00,
    BITS_16 = 0x01,
    BITS_17 = 0x02,
    BITS_18 = 0x03,
};

// 7 bit; datasheet family is 0x0C..0x0F
static constexpr uint8_t MLX90393_DEFAULT_ADDR = 0x18;

static constexpr uint8_t ZYXT = toByte(MLX90393_AXIS::X) | toByte(MLX90393_AXIS::Y) | toByte(MLX90393_AXIS::Z);
static constexpr uint32_t CONVERSION_MARGIN_MS = 10;
static constexpr uint32_t TRANSFER_TIMEOUT_MS = 100;
static constexpr uint32_t INIT_TIMEOUT_MS = 100;

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
static constexpr uint8_t CONFIG_X_RES = toByte(MLX90393_RES::BITS_16);
static constexpr uint8_t CONFIG_Y_RES = toByte(MLX90393_RES::BITS_16);
static constexpr uint8_t CONFIG_Z_RES = toByte(MLX90393_RES::BITS_15);

Magnetometer::Magnetometer(I2C_HandleTypeDef *hi2c, GPIO_TypeDef *csPort, uint16_t csPin)
    : Magnetometer(hi2c, csPort, csPin, MLX90393_DEFAULT_ADDR) {}

Magnetometer::Magnetometer(I2C_HandleTypeDef *hi2c, GPIO_TypeDef *csPort, uint16_t csPin, uint8_t address)
    : hi2c(hi2c),
      csPort(csPort),
      csPin(csPin),
      halAddress(static_cast<uint16_t>(address) << 1),
      gain(toByte(MLX90393_GAIN::X1)),
      xRes(CONFIG_X_RES),
      yRes(CONFIG_Y_RES),
      zRes(CONFIG_Z_RES),
      hallconf(HALLCONF_0XC),
      tcmpEn(0),
      filter(CONFIG_FILTER),
      osr(CONFIG_OSR),
      status(0),
      regValue(0),
      initialized(false),
      state(IDLE),
      sampleReady(false),
      pendingError(0),
      conversionDoneMs(0),
      transferStartMs(0),
      txByte(0),
      rxBuffer{0},
      magData{} {}

ZP_Error Magnetometer::init() {
    initialized = false;

    if (csPort != nullptr) {
        HAL_GPIO_WritePin(csPort, csPin, GPIO_PIN_SET);
        HAL_Delay(2);
    }

    if (HAL_I2C_IsDeviceReady(hi2c, halAddress, 2, INIT_TIMEOUT_MS) != HAL_OK) {
        return ZP_ERROR_EXT_API | ZP_ERROR_NOT_READY;
    }

    // Leaves any mode a previous boot left running. Fails harmlessly when idle.
    (void)sendCommand(toByte(MLX90393_CMD::EXIT));
    HAL_Delay(1);

    ZP_Error result = sendCommand(toByte(MLX90393_CMD::RESET));
    if (result != ZP_ERROR_OK) {
        return result;
    }
    HAL_Delay(2);

    result = refreshConfigCache();
    if (result != ZP_ERROR_OK) {
        return result;
    }

    result = setFilter(CONFIG_FILTER);
    if (result != ZP_ERROR_OK) {
        return result;
    }
    result = setOversampling(CONFIG_OSR);
    if (result != ZP_ERROR_OK) {
        return result;
    }
    result = setResolution(CONFIG_X_RES, CONFIG_Y_RES, CONFIG_Z_RES);
    if (result != ZP_ERROR_OK) {
        return result;
    }

    result = refreshConfigCache();
    if (result != ZP_ERROR_OK) {
        return result;
    }

    state = IDLE;
    sampleReady = false;
    pendingError = 0;
    magData = {};
    initialized = true;

    return ZP_ERROR_OK;
}

ZP_Error Magnetometer::transceive(uint8_t *txData, uint16_t txSize, uint8_t *rxData, uint16_t rxSize) {
    ZP_Error result = halToZpError(HAL_I2C_Master_Transmit(hi2c, halAddress, txData, txSize, INIT_TIMEOUT_MS));
    if (result != ZP_ERROR_OK) {
        return result;
    }
    return halToZpError(HAL_I2C_Master_Receive(hi2c, halAddress, rxData, rxSize, INIT_TIMEOUT_MS));
}

ZP_Error Magnetometer::sendCommand(uint8_t command) {
    uint8_t tx = command;
    uint8_t rx = 0;
    ZP_Error result = transceive(&tx, 1, &rx, 1);
    if (result != ZP_ERROR_OK) {
        return result;
    }
    status = rx;
    return ZP_ERROR_OK;
}

ZP_Error Magnetometer::writeRegister(uint8_t reg, uint16_t value) {
    uint8_t tx[4] = {
        toByte(MLX90393_CMD::WRITE_REGISTER),
        static_cast<uint8_t>(value >> 8),
        static_cast<uint8_t>(value & 0xFF),
        static_cast<uint8_t>(reg << 2),
    };
    uint8_t rx = 0;
    ZP_Error result = transceive(tx, 4, &rx, 1);
    if (result != ZP_ERROR_OK) {
        return result;
    }
    status = rx;
    return ZP_ERROR_OK;
}

ZP_Error Magnetometer::readRegister(uint8_t reg) {
    uint8_t tx[2] = {toByte(MLX90393_CMD::READ_REGISTER), static_cast<uint8_t>(reg << 2)};
    uint8_t rx[3] = {0};
    ZP_Error result = transceive(tx, 2, rx, 3);
    if (result != ZP_ERROR_OK) {
        return result;
    }
    status = rx[0];
    regValue = static_cast<uint16_t>(rx[1] << 8 | rx[2]);
    return ZP_ERROR_OK;
}

ZP_Error Magnetometer::refreshConfigCache() {
    ZP_Error result = readRegister(toByte(MLX90393_REG::CONF1));
    if (result != ZP_ERROR_OK) {
        return result;
    }
    gain = (regValue & GAIN_MASK) >> GAIN_SHIFT;
    hallconf = regValue & HALLCONF_MASK;
    tcmpEn = (regValue & TCMP_EN_MASK) >> TCMP_EN_SHIFT;

    result = readRegister(toByte(MLX90393_REG::CONF3));
    if (result != ZP_ERROR_OK) {
        return result;
    }
    xRes = (regValue & X_RES_MASK) >> X_RES_SHIFT;
    yRes = (regValue & Y_RES_MASK) >> Y_RES_SHIFT;
    zRes = (regValue & Z_RES_MASK) >> Z_RES_SHIFT;
    filter = (regValue & FILTER_MASK) >> FILTER_SHIFT;
    osr = (regValue & OSR_MASK) >> OSR_SHIFT;

    return ZP_ERROR_OK;
}

ZP_Error Magnetometer::setGain(uint8_t newGain) {
    ZP_Error result = readRegister(toByte(MLX90393_REG::CONF1));
    if (result != ZP_ERROR_OK) {
        return result;
    }
    const uint16_t VALUE = (regValue & ~GAIN_MASK) | (newGain << GAIN_SHIFT);
    result = writeRegister(toByte(MLX90393_REG::CONF1), VALUE);
    if (result != ZP_ERROR_OK) {
        return result;
    }
    gain = newGain;
    return ZP_ERROR_OK;
}

ZP_Error Magnetometer::setResolution(uint8_t newXRes, uint8_t newYRes, uint8_t newZRes) {

    if (tcmpEn == 1 &&
        (newXRes >= toByte(MLX90393_RES::BITS_17) || newYRes >= toByte(MLX90393_RES::BITS_17) || newZRes >= toByte(MLX90393_RES::BITS_17))) {
        return ZP_ERROR_INVALID_ARG;
    }

    ZP_Error result = readRegister(toByte(MLX90393_REG::CONF3));
    if (result != ZP_ERROR_OK) {
        return result;
    }
    uint16_t value = regValue;
    value = (value & ~X_RES_MASK) | (newXRes << X_RES_SHIFT);
    value = (value & ~Y_RES_MASK) | (newYRes << Y_RES_SHIFT);
    value = (value & ~Z_RES_MASK) | (newZRes << Z_RES_SHIFT);

    result = writeRegister(toByte(MLX90393_REG::CONF3), value);
    if (result != ZP_ERROR_OK) {
        return result;
    }
    xRes = newXRes;
    yRes = newYRes;
    zRes = newZRes;
    return ZP_ERROR_OK;
}

ZP_Error Magnetometer::setFilter(uint8_t newFilter) {

    if (hallconf == HALLCONF_0XC) {
        if (osr == 0x00 && (newFilter == 0 || newFilter == 1)) {
            return ZP_ERROR_INVALID_ARG;
        }
        if (osr == 0x01 && newFilter == 0) {
            return ZP_ERROR_INVALID_ARG;
        }
    }

    ZP_Error result = readRegister(toByte(MLX90393_REG::CONF3));
    if (result != ZP_ERROR_OK) {
        return result;
    }
    const uint16_t VALUE = (regValue & ~FILTER_MASK) | (newFilter << FILTER_SHIFT);
    result = writeRegister(toByte(MLX90393_REG::CONF3), VALUE);
    if (result != ZP_ERROR_OK) {
        return result;
    }
    filter = newFilter;
    return ZP_ERROR_OK;
}

ZP_Error Magnetometer::setOversampling(uint8_t newOsr) {

    if (hallconf == HALLCONF_0XC) {
        if (filter == 0x00 && (newOsr == 0 || newOsr == 1)) {
            return ZP_ERROR_INVALID_ARG;
        }
        if (filter == 0x01 && newOsr == 0) {
            return ZP_ERROR_INVALID_ARG;
        }
    }

    ZP_Error result = readRegister(toByte(MLX90393_REG::CONF3));
    if (result != ZP_ERROR_OK) {
        return result;
    }
    const uint16_t VALUE = (regValue & ~OSR_MASK) | (newOsr << OSR_SHIFT);
    result = writeRegister(toByte(MLX90393_REG::CONF3), VALUE);
    if (result != ZP_ERROR_OK) {
        return result;
    }
    osr = newOsr;
    return ZP_ERROR_OK;
}

ZP_Error Magnetometer::readData(MagData_t &data) {
    if (!initialized) {
        magData.isNew = false;
        data = magData;
        return ZP_ERROR_NOT_READY;
    }

    ZP_Error result(pendingError);
    pendingError = 0;

    if (sampleReady) {
        sampleReady = false;
        result |= decode();
    } else {
        magData.isNew = false;
    }

    const uint32_t NOW = HAL_GetTick();

    switch (state) {
        case IDLE:
            result |= startMeasurement();
            break;
        case WAIT_CONV:
            if (static_cast<int32_t>(NOW - conversionDoneMs) >= 0) {
                result |= readMeasurement();
            }
            break;
        default:
            if (NOW - transferStartMs > TRANSFER_TIMEOUT_MS &&
                hi2c->State == HAL_I2C_STATE_READY) {
                state = IDLE;
                result |= ZP_ERROR_TIMEOUT;
            }
            break;
    }

    data = magData;
    return result;
}

ZP_Error Magnetometer::startMeasurement() {
    txByte = toByte(MLX90393_CMD::START_MEASUREMENT) | ZYXT;
    transferStartMs = HAL_GetTick();
    state = SM_TX;

    const ZP_Error STATUS = halToZpError(HAL_I2C_Master_Transmit_IT(hi2c, halAddress, &txByte, 1));
    if (STATUS != ZP_ERROR_OK) {
        state = IDLE;
    }
    return STATUS;
}

ZP_Error Magnetometer::readMeasurement() {
    txByte = toByte(MLX90393_CMD::READ_MEASUREMENT) | ZYXT;
    transferStartMs = HAL_GetTick();
    state = RM_TX;

    const ZP_Error STATUS = halToZpError(HAL_I2C_Master_Transmit_IT(hi2c, halAddress, &txByte, 1));
    if (STATUS != ZP_ERROR_OK) {
        state = IDLE;
    }
    return STATUS;
}

void Magnetometer::masterTxCpltCallback() {
    switch (state) {
        case SM_TX:
            state = SM_RX;

            if (HAL_I2C_Master_Receive_IT(hi2c, halAddress, &status, 1) != HAL_OK) {
                state = IDLE;
                pendingError |= (ZP_ERROR_EXT_API | ZP_ERROR_FAIL).raw();
            }
            break;
        case RM_TX:
            state = RM_RX;
            if (HAL_I2C_Master_Receive_IT(hi2c, halAddress, rxBuffer, MEASUREMENT_BYTES) != HAL_OK) {
                state = IDLE;
                pendingError |= (ZP_ERROR_EXT_API | ZP_ERROR_FAIL).raw();
            }
            break;
        default:
            break;
    }
}

void Magnetometer::masterRxCpltCallback() {
    switch (state) {
        case SM_RX:
            if (status & toByte(MLX90393_STATUS::ERROR)) {
                state = IDLE;
                pendingError |= ZP_ERROR_INVALID_DATA.raw();
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
    const bool NACKED = (HAL_I2C_GetError(hi2c) & HAL_I2C_ERROR_AF) != 0;
    pendingError |= (ZP_ERROR_EXT_API | (NACKED ? ZP_ERROR_NACK : ZP_ERROR_FAIL)).raw();
    state = IDLE;
}

uint32_t Magnetometer::conversionTimeMs() const {
    const float TCONV = TCONV_MS[filter & 0x07][osr & 0x03];
    return static_cast<uint32_t>(TCONV) + 1 + CONVERSION_MARGIN_MS;
}

ZP_Error Magnetometer::decode() {
    const uint8_t SAMPLE_STATUS = rxBuffer[0];

    if (SAMPLE_STATUS & toByte(MLX90393_STATUS::ERROR)) {
        magData.isNew = false;
        return ZP_ERROR_INVALID_DATA;
    }

    const uint8_t *cursor = rxBuffer + 1;

    int16_t raw[3] = {0, 0, 0};

    if (ZYXT & toByte(MLX90393_AXIS::T)) {
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
            if (RES[i] == toByte(MLX90393_RES::BITS_17)) {
                counts[i] = static_cast<int32_t>(static_cast<uint16_t>(raw[i])) - RES17_ZERO_OFFSET;
            } else if (RES[i] == toByte(MLX90393_RES::BITS_18)) {
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

    magData.x = field[0];
    magData.y = field[1];
    magData.z = field[2];
    magData.rawX = raw[0];
    magData.rawY = raw[1];
    magData.rawZ = raw[2];
    magData.timestamp = NOW;
    magData.isNew = true;
    return ZP_ERROR_OK;
}

I2C_HandleTypeDef *Magnetometer::getI2C() {
    return hi2c;
}
