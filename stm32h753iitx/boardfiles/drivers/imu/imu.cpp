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
#define FIFO_HEADER_MSG_BIT 0x80
#define FIFO_HEADER_ACCEL_BIT 0x40
#define FIFO_HEADER_GYRO_BIT 0x20

#define ICM42688P_IMU_WHOAMI 0x47

IMU::IMU(SPI_HandleTypeDef *spiHandle, GPIO_TypeDef *csPort, uint16_t csPin, uint8_t imuId) : 
    spi(spiHandle),
    csPort(csPort),
    csPin(csPin),
    imuId(imuId),
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
        scaledData[i].xgyro = lowPassFilter((float)rawDataBatch.data[i].xgyro / GYRO_SEN_SCALE_FACTOR, 0);
        scaledData[i].ygyro = lowPassFilter((float)rawDataBatch.data[i].ygyro / GYRO_SEN_SCALE_FACTOR, 1);
        scaledData[i].zgyro = lowPassFilter((float)rawDataBatch.data[i].zgyro / GYRO_SEN_SCALE_FACTOR, 2);
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
    uint8_t buffer;
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
    writeRegister(0, UB0_REG_GYRO_ODR, 0b00000100); // Configure gyro ODR to 4khz
    writeRegister(0, UB0_REG_ACCEL_CONFIG0, 0b00000100); // Configure accelerometer ODR to 4khz
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

        rawData[k].xacc = (int16_t)((imuRxBuffer[base + 1] << 8) | imuRxBuffer[base + 2]);
        rawData[k].yacc = -(int16_t)((imuRxBuffer[base + 3] << 8) | imuRxBuffer[base + 4]);
        rawData[k].zacc = (int16_t)((imuRxBuffer[base + 5] << 8) | imuRxBuffer[base + 6]);
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

float IMU::getODR() {
    return 4000.0f; // Change when using a different ODR
}