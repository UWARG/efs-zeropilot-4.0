// IMU.cpp
#include "imu.hpp"
#include "systemutils.hpp"
#include <string.h>

#define REG_BANK_SEL              0x76
#define UB0_REG_WHO_AM_I          0x75
#define UB0_REG_DEVICE_CONFIG     0x11
#define UB0_REG_PWR_MGMT0         0x4E
#define UB0_REG_FIFO_CONFIG       0x16
#define UB0_REG_FIFO_CONFIG1      0x5F
#define UB0_REG_INTF_CONFIG0      0x4C
#define UB0_REG_FIFO_DATA         0x30
#define UB0_REG_FIFO_COUNTH       0x2E
#define UB0_REG_SIGNAL_PATH_RESET 0x4B
#define UB0_REG_GYRO_ODR          0x4F
#define UB0_REG_ACCEL_CONFIG0     0x50
#define UB0_REG_GYRO_CONFIG1      0x51
#define UB0_REG_ACCEL_CONFIG1      0x53
#define UB0_REG_GYRO_ACCEL_CONFIG0 0x52
#define UB1_REG_GYRO_CONFIG_STATIC2 0x0B
#define UB1_REG_GYRO_CONFIG_STATIC3 0x0C
#define UB1_REG_GYRO_CONFIG_STATIC4 0x0D
#define UB1_REG_GYRO_CONFIG_STATIC5 0x0E
#define UB2_REG_ACCEL_CONFIG_STATIC2 0x03
#define UB2_REG_ACCEL_CONFIG_STATIC3 0x04
#define UB2_REG_ACCEL_CONFIG_STATIC4 0x05

#define FIFO_HEADER_MSG_BIT 0x80
#define FIFO_HEADER_ACCEL_BIT 0x40
#define FIFO_HEADER_GYRO_BIT 0x20

#define ICM42688P_IMU_WHOAMI 0x47

typedef struct {
	uint16_t bandwidth;
	uint8_t delt;
	uint16_t deltsqr;
	uint8_t bitshift;
} AAF_Config;

static const AAF_Config aaf_table[] = {
    {42,   1,    1,   15}, {84,   2,    4,   13}, {126,  3,    9,   12},
    {170,  4,   16,   11}, {213,  5,   25,   10}, {258,  6,   36,   10},
    {303,  7,   49,    9}, {348,  8,   64,    9}, {394,  9,   81,    9},
    {441, 10,  100,    8}, {488, 11,  122,    8}, {536, 12,  144,    8},
    {585, 13,  170,    8}, {634, 14,  196,    7}, {684, 15,  224,    7},
    {734, 16,  256,    7}, {785, 17,  288,    7}, {837, 18,  324,    7},
    {890, 19,  360,    6}, {943, 20,  400,    6}, {997, 21,  440,    6},
    {1051,22,  488,    6}, {1107,23,  528,    6}, {1163,24,  576,    6},
    {1220,25,  624,    6}, {1277,26,  680,    6}, {1336,27,  736,    5},
    {1395,28,  784,    5}, {1454,29,  848,    5}, {1515,30,  896,    5},
    {1577,31,  960,    5}, {1639,32, 1024,    5}, {1702,33, 1088,    5},
    {1766,34, 1152,    5}, {1830,35, 1232,    5}, {1896,36, 1296,    5},
    {1962,37, 1376,    4}, {2029,38, 1440,    4}, {2097,39, 1536,    4},
    {2166,40, 1600,    4}, {2235,41, 1696,    4}, {2306,42, 1760,    4},
    {2377,43, 1856,    4}, {2449,44, 1952,    4}, {2522,45, 2016,    4},
    {2596,46, 2112,    4}, {2671,47, 2208,    4}, {2746,48, 2304,    4},
    {2823,49, 2400,    4}, {2900,50, 2496,    4}, {2978,51, 2592,    4},
    {3057,52, 2720,    4}, {3137,53, 2816,    3}, {3217,54, 2944,    3},
    {3299,55, 3008,    3}, {3381,56, 3136,    3}, {3464,57, 3264,    3},
    {3548,58, 3392,    3}, {3633,59, 3456,    3}, {3718,60, 3584,    3},
    {3805,61, 3712,    3}, {3892,62, 3840,    3}, {3979,63, 3968,    3}
};

IMU::IMU(SPI_HandleTypeDef *spiHandle, 
        GPIO_TypeDef *csPort, 
        uint16_t csPin, 
        uint8_t imuId, 
        ImuOdrConfig_t odrConfig,
        float uiFiltCutoffHz, 
        ImuUiFiltOrder_t uiFiltOrder
) :
    spi(spiHandle),
    csPort(csPort),
    csPin(csPin),
    imuId(imuId),
    imuOdr(odrConfig),
    uiFiltCutoffHz(uiFiltCutoffHz),
    uiFiltOrder(uiFiltOrder),
    alpha(0.1f) {

    filteredGyro[0] = filteredGyro[1] = filteredGyro[2] = 0.0f;
    memset((void *)imuTxBuffer, 0, RX_BUFFER_SIZE);
    memset((void *)imuRxBuffer, 0, RX_BUFFER_SIZE);

    // First bit should be 1 for register read
    imuTxBuffer[0] = UB0_REG_FIFO_DATA | 0b10000000;
}

int IMU::init() {
    csHigh();
    SystemUtils::dwtInit();
    reset();
    uint8_t address = whoAmI();
    setAAF();
    setUIFilt();
    setODR();
    setFIFO();
    flushFIFO();
    setLowNoiseMode();
    HAL_Delay(60); // Wait after sensors are turned on

    return (address == ICM42688P_IMU_WHOAMI) ? 0 : -1;
}

RawImuBatch_t IMU::readRawData() {
    // Dont start another dma transaction when in the middle of one transaction
    if (!dmaDone) {
        rawImuDataBatch.count = 0;
        return rawImuDataBatch;
    }
    setBank(0);
    RawImuBatch_t batch = getBatch();
    beginRead();
    return batch;
}

ScaledImuBatch_t IMU::scaleIMUData(const RawImuBatch_t &rawDataBatch) {
    for (int i = 0; i < rawDataBatch.count; i++) {
        scaledData[i].xacc = (float)rawDataBatch.data[i].xacc / ACCEL_SEN_SCALE_FACTOR;
        scaledData[i].yacc = (float)rawDataBatch.data[i].yacc / ACCEL_SEN_SCALE_FACTOR;
        scaledData[i].zacc = (float)rawDataBatch.data[i].zacc / ACCEL_SEN_SCALE_FACTOR;
        scaledData[i].xgyro = (float)rawDataBatch.data[i].xgyro / GYRO_SEN_SCALE_FACTOR;
        scaledData[i].ygyro = (float)rawDataBatch.data[i].ygyro / GYRO_SEN_SCALE_FACTOR;
        scaledData[i].zgyro = (float)rawDataBatch.data[i].zgyro / GYRO_SEN_SCALE_FACTOR;
        scaledData[i].timestamp = rawDataBatch.data[i].timestamp;
    }
    scaledImuDataBatch.count = rawDataBatch.count;
    scaledImuDataBatch.data = scaledData;
    scaledImuDataBatch.readTime = rawImuDataBatch.readTime;

    return scaledImuDataBatch;
}

void IMU::txRxCallback() {
    csHigh();
    switch (rxFlag) {
    case COUNT:
        // Keeps the bus owned
        rxFlag = DATA;
        dmaTransfer(); // Read actual data after getting num of packets
        break;
    case DATA:
        // Free the bus as fifo read is completed
        rxFlag = COUNT;
        dmaDone = true;
        break;
    default:
        break;
    }
}

SPI_HandleTypeDef *IMU::getSPI() {
    return spi;
}

bool IMU::getDmaFlag() {
    return dmaDone;
}

void IMU::beginRead() {
    // Dont start another dma transaction when in the middle of one transaction
    if (!dmaDone) {
        return;
    }

    // Start another batch transfer
    setBank(0);
    dmaDone = false;
    rxFlag = COUNT;
    dmaTransfer();
}

RawImuBatch_t IMU::getBatch() {
    processRawData();
    return rawImuDataBatch;
}

HAL_StatusTypeDef IMU::writeRegister(uint8_t bank, uint8_t registerAddr, uint8_t data) {
    HAL_StatusTypeDef status = setBank(bank);
    if (status != HAL_OK) {
        return status;
    }
    uint8_t txBuf[2] = {registerAddr, data};
    csLow();
    status = HAL_SPI_Transmit(spi, txBuf, 2, HAL_MAX_DELAY);
    csHigh();
    return status;
}

HAL_StatusTypeDef IMU::readRegister(uint8_t bank, uint8_t registerAddr, uint8_t* data) {
    HAL_StatusTypeDef status = setBank(bank);
    if (status != HAL_OK) {
        return status;
    }

    uint8_t tx[2] = {(uint8_t)(registerAddr | 0b10000000), 0}; // Set 8-th bit to 1 for read, page 53
    uint8_t rx[2] = {0, 0};

    csLow();
    status = HAL_SPI_TransmitReceive(spi, tx, rx, 2, HAL_MAX_DELAY);
    csHigh();

    *data = rx[1];

    return status;
}

HAL_StatusTypeDef IMU::setBank(uint8_t bank) {
    if (currRegisterBank == bank) {
        return HAL_OK;
    }
    uint8_t txBuf[2] = {REG_BANK_SEL, bank};
    csLow();
    HAL_StatusTypeDef status = HAL_SPI_Transmit(spi, txBuf, 2, HAL_MAX_DELAY);
    csHigh();

    currRegisterBank = bank;
    return status;
}

void IMU::csLow() {
    HAL_GPIO_WritePin(csPort, csPin, GPIO_PIN_RESET);
}

void IMU::csHigh() {
    HAL_GPIO_WritePin(csPort, csPin, GPIO_PIN_SET);
}

void IMU::reset() {
    writeRegister(0, UB0_REG_DEVICE_CONFIG, 0x01);
    HAL_Delay(1); // Need one ms delay after reset, 
}

uint8_t IMU::whoAmI() {
    uint8_t buffer = 0;
    readRegister(0, UB0_REG_WHO_AM_I, &buffer);
    return buffer;
}

void IMU::flushFIFO() {
    writeRegister(0, UB0_REG_SIGNAL_PATH_RESET, 0b00000010);
}

void IMU::dmaTransfer() {
    csLow();
    switch (rxFlag) {
        case COUNT: {
            imuTxBuffer[0] = UB0_REG_FIFO_COUNTH | 0b10000000;
            // 3 bytes to read both COUNTH and COUNTL registers, byte 0 is dummy
            if (HAL_SPI_TransmitReceive_DMA(spi, (uint8_t*)imuTxBuffer, (uint8_t*)imuRxBuffer, 3) != HAL_OK) {
                csHigh();
                dmaDone = true; // Allow next transfer to be attempted
                rxFlag = COUNT; // Reset state to COUNT
                fifoSize = 0;
            }
            break;
        }

        case DATA: {
            fifoSize = ((uint16_t)imuRxBuffer[1] << 8) | imuRxBuffer[2]; // [0] is the dummy byte
            if (fifoSize > MAX_PACKETS) { fifoSize = MAX_PACKETS; }
            
            imuTxBuffer[0] = UB0_REG_FIFO_DATA | 0b10000000;

            if (HAL_SPI_TransmitReceive_DMA(spi, (uint8_t*)imuTxBuffer, (uint8_t*)imuRxBuffer, fifoSize * PACKET_SIZE + 1) != HAL_OK) {
                csHigh();
                dmaDone = true; // Allow next transfer to be attempted
                rxFlag = COUNT; // Reset state to COUNT
                fifoSize = 0;
            }
            break;
        }

        default:
            break;
    }
}

void IMU::setLowNoiseMode() {
    // Starts accelerometer and gyro in low noise mode
    writeRegister(0, UB0_REG_PWR_MGMT0, 0x0F);
}

void IMU::setFIFO() {
    writeRegister(0, UB0_REG_FIFO_CONFIG, 0b01000000);  // Stream to fifo mode
    writeRegister(0, UB0_REG_FIFO_CONFIG1, 0b01100011); // Partial fifo read enabled, trigger watermark interrupt on every odr if count > watermark, no fsync, no temp data, yes gyro, yes accel
    writeRegister(0, UB0_REG_INTF_CONFIG0, 0b11110000); // Invalid data not put into fifo, fifo count is in num of packets
}

void IMU::setODR() {
    writeRegister(0, UB0_REG_GYRO_ODR, (uint8_t)imuOdr); // Configure gyro ODR to 4khz
    writeRegister(0, UB0_REG_ACCEL_CONFIG0, (uint8_t)imuOdr); // Configure accelerometer ODR to 4khz
}

void IMU::setAAF() {
    uint16_t desiredBandwidth = getODRHz() / 4; // Set AAF bandwidth to 1/4 of ODR
    uint8_t bestIndex = 0;
    uint16_t bestDistance = UINT16_MAX;
    // Table is sorted ascending, so distance falls to a minimum then rises again.
    // Stop as soon as it stops improving. Ties keep the lower bandwidth.
    for (uint8_t i = 0; i < sizeof(aaf_table) / sizeof(aaf_table[0]); i++) {
        uint16_t bandwidth = aaf_table[i].bandwidth;
        uint16_t distance = (bandwidth > desiredBandwidth) ? (bandwidth - desiredBandwidth) : (desiredBandwidth - bandwidth);
        if (distance >= bestDistance) {
            break;
        }
        bestDistance = distance;
        bestIndex = i;
    }
    uint8_t delt = (uint8_t)(aaf_table[bestIndex].delt & 0b00111111); // only 6 bits are used for delt
    uint8_t deltsqrLower = (uint8_t)aaf_table[bestIndex].deltsqr;
    uint8_t deltsqrUpper = (uint8_t)((aaf_table[bestIndex].deltsqr >> 8) & 0xF); // only 4 bits are used for upper deltsqr
    uint8_t bitshift = (uint8_t)(aaf_table[bestIndex].bitshift & 0xF); // only 4 bits are used for bitshift
    
    writeRegister(2, UB2_REG_ACCEL_CONFIG_STATIC2, (delt << 1)); // 6:1 for ACCEL_AAF_DELT, bit 0 is disable accel AAF, defualt enabled
    writeRegister(2, UB2_REG_ACCEL_CONFIG_STATIC3, deltsqrLower); // ACCEL_AAF_DELTSQR
    writeRegister(2, UB2_REG_ACCEL_CONFIG_STATIC4, (bitshift << 4) | deltsqrUpper); // ACCEL_AAF_BITSHIFT

    writeRegister(1, UB1_REG_GYRO_CONFIG_STATIC3, delt); // 5:0 for GYRO_AAF_DELT, 7:6 reserved
    writeRegister(1, UB1_REG_GYRO_CONFIG_STATIC4, deltsqrLower); // GYRO_AAF_DELTSQR
    writeRegister(1, UB1_REG_GYRO_CONFIG_STATIC5, (bitshift << 4) | deltsqrUpper); // GYRO_AAF_BITSHIFT
    writeRegister(1, UB1_REG_GYRO_CONFIG_STATIC2, 0b00000001); // Enable gyro AAF and disables notch filter
}

float IMU::getUIFiltBWHz(uint8_t bandwidthSelect) {
    // Bandwidth = max(400Hz, ODR) / divisor, except setting 0 which is ODR/2
    // Values 8-13 are reserved and 14-15 are low latency modes, so only 0-7 are valid
    static const uint8_t divisors[UIFILT_BW_SEL_COUNT] = {2, 4, 5, 8, 10, 16, 20, 40};
    if (bandwidthSelect >= UIFILT_BW_SEL_COUNT) {
        return 0.0f;
    }
    float odr = getODRHz();
    float max = (bandwidthSelect == 0 || odr > 400.0f) ? odr : 400.0f;
    return max / divisors[bandwidthSelect];
}

void IMU::setUIFilt() {
    // 7:5 TEMP_FILT_BW = 0 (default), bit 4 reserved (reset value 1)
    // 3:2 GYRO_UI_FILT_ORD, 1:0 GYRO_DEC2_M2_ORD = 0b10 (3rd order, only valid setting)
    writeRegister(0, UB0_REG_GYRO_CONFIG1, (uint8_t)(0b00010000 | (uiFiltOrder << 2) | 0b10));
    // 7:5 reserved (reset value 0), 4:3 ACCEL_UI_FILT_ORD
    // 2:1 ACCEL_DEC2_M2_ORD = 0b10 (3rd order, only valid setting), bit 0 reserved (reset value 1)
    writeRegister(0, UB0_REG_ACCEL_CONFIG1, (uint8_t)((uiFiltOrder << 3) | 0b101));

    // Find the closest bandwidth to the requested cutoff frequency
    uint8_t bestSel = 0; // Value for GYRO_UI_FILT_BW/ACCEL_UI_FILT_BW register (0-7)
    float bestDistance = -1.0f;
    for (uint8_t bwSel = 0; bwSel < UIFILT_BW_SEL_COUNT; bwSel++) {
        float distance = getUIFiltBWHz(bwSel) - uiFiltCutoffHz;
        if (distance < 0.0f) {
            distance = -distance;
        }
        if (bestDistance < 0.0f || distance < bestDistance) {
            bestDistance = distance;
            bestSel = bwSel;
        }
    }

    // 7:4 ACCEL_UI_FILT_BW, 3:0 GYRO_UI_FILT_BW
    writeRegister(0, UB0_REG_GYRO_ACCEL_CONFIG0, (uint8_t)((bestSel << 4) | bestSel));
}

void IMU::processRawData() {
    uint16_t validData = 0;
    for (int k = 0; k < fifoSize; k++)
    {
        uint16_t base = 1 + k * PACKET_SIZE; // +1 to skip the dummy byte

        uint8_t header = imuRxBuffer[base];
        // Dont read data if the packet is empty or if doesnt include acceleration or gyro data
        if ((header & FIFO_HEADER_MSG_BIT) || !(header & FIFO_HEADER_ACCEL_BIT) || !(header & FIFO_HEADER_GYRO_BIT)) {
            break;
        }
        // FRD
        rawData[k].xacc = -(int16_t)((imuRxBuffer[base + 1] << 8) | imuRxBuffer[base + 2]);
        rawData[k].yacc = (int16_t)((imuRxBuffer[base + 3] << 8) | imuRxBuffer[base + 4]);
        rawData[k].zacc = -(int16_t)((imuRxBuffer[base + 5] << 8) | imuRxBuffer[base + 6]);
        rawData[k].xgyro = -(int16_t)((imuRxBuffer[base + 7] << 8) | imuRxBuffer[base + 8]);
        rawData[k].ygyro = (int16_t)((imuRxBuffer[base + 9] << 8) | imuRxBuffer[base + 10]);
        rawData[k].zgyro = -(int16_t)((imuRxBuffer[base + 11] << 8) | imuRxBuffer[base + 12]);
        rawData[k].timestamp = (uint16_t)((imuRxBuffer[base + 14] << 8) | imuRxBuffer[base + 15]);
        rawData[k].imuId = imuId;
        validData++;
    }

    rawImuDataBatch.data = rawData;
    rawImuDataBatch.count = validData;
    rawImuDataBatch.readTime = SystemUtils::getDWTMicroSec();
}

float IMU::lowPassFilter(float rawValue, int select) {
    filteredGyro[select] = alpha * rawValue + (1 - alpha) * filteredGyro[select];
    return filteredGyro[select];
}

float IMU::getODRHz() {
    switch (imuOdr) {
        case IMU_ODR_32KHZ: return 32000.0f;
        case IMU_ODR_16KHZ: return 16000.0f;
        case IMU_ODR_8KHZ:  return 8000.0f;
        case IMU_ODR_4KHZ:  return 4000.0f;
        case IMU_ODR_2KHZ:  return 2000.0f;
        case IMU_ODR_1KHZ:  return 1000.0f;
        case IMU_ODR_500HZ: return 500.0f;
        case IMU_ODR_200HZ: return 200.0f;
        case IMU_ODR_100HZ: return 100.0f;
        case IMU_ODR_50HZ:  return 50.0f;
        case IMU_ODR_25HZ:  return 25.0f;
        case IMU_ODR_12HZ5: return 12.5f;
        default:            return 0.0f;
    }
}
