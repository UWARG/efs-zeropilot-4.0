// IMU.cpp

#include "imu.hpp"
#include <string.h>

#define REG_BANK_SEL          0x76
#define UB0_REG_WHO_AM_I      0x75
#define UB0_REG_DEVICE_CONFIG 0x11
#define UB0_REG_PWR_MGMT0     0x4E
#define UB0_REG_FIFO_CONFIG       0x16
#define UB0_REG_FIFO_CONFIG1      0x5F
#define UB0_REG_INTF_CONFIG0      0x4C
#define UB0_REG_FIFO_DATA         0x30
#define UB0_REG_FIFO_COUNTH       0x2E
#define UB0_REG_SIGNAL_PATH_RESET 0x4B

IMU::IMU(SPI_HandleTypeDef* spiHandle, GPIO_TypeDef* csPort, uint16_t csPin) : 
    _spi(spiHandle), 
    _csPort(csPort), 
    _csPin(csPin),
    _alpha(0.1f) {

    _filteredGyro[0] = _filteredGyro[1] = _filteredGyro[2] = 0.0f;
    memset((void*)imuTxBuffer, 0, RX_BUFFER_SIZE);
    memset((void*)imuRxBuffer, 0, RX_BUFFER_SIZE);
    
    // First bit should be 1 for register read
    imuTxBuffer[0] = UB0_REG_FIFO_DATA | 0b10000000;
}

int IMU::init() {
    csHigh();
    reset();
    uint8_t address = whoAmI();
    setLowNoiseMode();
    HAL_Delay(60); // Wait after sensors are turned on
    setFIFO();
    flushFIFO();

    // TODO: enable and test below configurations
    // setAccelFS(0b01101001);
    // configureNotchFilter();
    // setAntiAliasFilter(213, true, true);
    // calibrateGyro();
    return address;
}

RawImuBatch_t IMU::readRawData() {
    // Dont start another dma transaction when in the middle of one transaction
    if (!dmaDone) {
        return rawImuDataBatch;
    }
    
    // Process previous data batch    
    processRawData(); 

    // Start another batch transfer
    setBank(0);
    dmaDone = false;
    rxFlag = COUNT;
    dmaTransfer();

    return rawImuDataBatch;
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

    return scaledImuDataBatch;
}

void IMU::txRxCallback() {
    csHigh();
    switch (rxFlag) {
        case COUNT:
            rxFlag = DATA;
            dmaTransfer(); // Read actual data after getting num of packets
            break;
        case DATA:
            rxFlag = COUNT;
            dmaDone = true;
            break;
        default:
            break;
    }
}

SPI_HandleTypeDef* IMU::getSPI() {
    return _spi;
}

HAL_StatusTypeDef IMU::writeRegister(uint8_t bank, uint8_t registerAddr, uint8_t data) {
    HAL_StatusTypeDef status = setBank(bank);
    if (status != HAL_OK) {
        return status;
    }
    uint8_t txBuf[2] = {registerAddr, data};
    csLow();
    status = HAL_SPI_Transmit(_spi, txBuf, 2, HAL_MAX_DELAY);
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
    status = HAL_SPI_TransmitReceive(_spi, tx, rx, 2, HAL_MAX_DELAY);
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
    HAL_StatusTypeDef status = HAL_SPI_Transmit(_spi, txBuf, 2, HAL_MAX_DELAY);
    csHigh();

    currRegisterBank = bank;
    return status;
}

void IMU::csLow() {
    HAL_GPIO_WritePin(_csPort, _csPin, GPIO_PIN_RESET);
}

void IMU::csHigh() {
    HAL_GPIO_WritePin(_csPort, _csPin, GPIO_PIN_SET);
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
        case COUNT:
            imuTxBuffer[0] = UB0_REG_FIFO_COUNTH | 0b10000000;

            // 3 bytes to read both COUNTH and COUNTL registers, byte 0 is dummy
            if (HAL_SPI_TransmitReceive_DMA(_spi, (uint8_t*)imuTxBuffer, (uint8_t*)imuRxBuffer, 3) != HAL_OK) {
                csHigh();
                dmaDone = true; // Allow next transfer to be attempted
                rxFlag = COUNT; // Reset state to COUNT
            }
            break;

        case DATA:
            fifoSize = ((uint16_t)imuRxBuffer[1] << 8) | imuRxBuffer[2]; // [0] is the dummy byte
            if (fifoSize > MAX_PACKETS) { fifoSize = MAX_PACKETS; }

            imuTxBuffer[0] = UB0_REG_FIFO_DATA | 0b10000000;
            if (HAL_SPI_TransmitReceive_DMA(_spi, (uint8_t*)imuTxBuffer, (uint8_t*)imuRxBuffer, fifoSize * PACKET_SIZE + 1) != HAL_OK) {
                csHigh();
                dmaDone = true; // Allow next transfer to be attempted
                rxFlag = COUNT; // Reset state to COUNT
            }
            break;

        default:
            break;
    }
}

void IMU::setLowNoiseMode() {
    // Starts accelerometer and gyro in low noise mode
    writeRegister(0, UB0_REG_PWR_MGMT0, 0x0F);
}

void IMU::setFIFO() {
    // change to 0x later
    writeRegister(0, UB0_REG_FIFO_CONFIG, 0b01000000); // Stream to fifo mode
    writeRegister(0, UB0_REG_FIFO_CONFIG1, 0b01100011); // Partial fifo read enabled, trigger watermark interrupt on every odr if count > watermark, no fsync, no temp data, yes gyro, yes accel
    writeRegister(0, UB0_REG_INTF_CONFIG0, 0b11110000); // Invalid data not put into fifo, fifo count is in num of packets
}

void IMU::processRawData() {
    for (int k = 0; k < fifoSize; k++) {
        uint16_t base = 1 + k * PACKET_SIZE; // +1 to skip the dummy byte

        rawData[k].xacc = (int16_t)((imuRxBuffer[base+1] << 8) | imuRxBuffer[base+2]);
        rawData[k].yacc = -(int16_t)((imuRxBuffer[base+3] << 8) | imuRxBuffer[base+4]);
        rawData[k].zacc = (int16_t)((imuRxBuffer[base+5] << 8) | imuRxBuffer[base+6]);
        rawData[k].xgyro = -(int16_t)((imuRxBuffer[base+7] << 8) | imuRxBuffer[base+8]);
        rawData[k].ygyro = (int16_t)((imuRxBuffer[base+9] << 8) | imuRxBuffer[base+10]);
        rawData[k].zgyro = -(int16_t)((imuRxBuffer[base+11] << 8) | imuRxBuffer[base+12]);
        rawData[k].timestamp = (int16_t)((imuRxBuffer[base+14] << 8) | imuRxBuffer[base+15]);
    }

    rawImuDataBatch.data = rawData;
    rawImuDataBatch.count = fifoSize;

    // float acc_temp[3];
    // float gyr_temp[3];

    // acc_temp[0] = (float)raw[1] / 2048.0f * 9.81f;
    // acc_temp[1] = (float)raw[2] / 2048.0f * 9.81f;
    // acc_temp[2] = (float)raw[3] / 2048.0f * 9.81f;

    // gyr_temp[0] = lowPassFilter((float)raw[4] / 16.4f, 0);
    // gyr_temp[1] = lowPassFilter((float)raw[5] / 16.4f, 1);
    // gyr_temp[2] = lowPassFilter((float)raw[6] / 16.4f, 2);

    // // NED
    // rawImuDataBatch.xacc = (float)acc_temp[1];
    // rawImuDataBatch.yacc = (float)acc_temp[0];
    // rawImuDataBatch.zacc = ((float)acc_temp[2]);
    // rawImuDataBatch.xgyro = ((float)-gyr_temp[1]);
    // rawImuDataBatch.ygyro = ((float)-gyr_temp[0]);
    // rawImuDataBatch.zgyro = ((float)-gyr_temp[2]);
}

float IMU::lowPassFilter(float rawValue, int select) {
    _filteredGyro[select] = _alpha * rawValue + (1 - _alpha) * _filteredGyro[select];
    return _filteredGyro[select];
}

// TODO: verify correctness of below functions
/*
IMU::IMU(SPI_HandleTypeDef* spiHandle, GPIO_TypeDef* csPort, uint16_t csPin)
    : _spi(spiHandle), _csPort(csPort), _csPin(csPin),
      _gyroScale(0), _accelScale(0), _gyroFS(0), _accelFS(0),
      _alpha(0.1f)
{
    _gyrB[0] = _gyrB[1] = _gyrB[2] = 0.0f;
    _accB[0] = _accB[1] = _accB[2] = 0.0f;
    _filteredGyro[0] = _filteredGyro[1] = _filteredGyro[2] = 0.0f;
}


void IMU::setGyroFS(uint8_t fssel) {
    setBank(0);
    uint8_t reg;
    readRegisters(0x4F, 1, &reg);
    reg = (fssel << 5) | (reg & 0x1F);
    writeRegister(0x4F, reg);
    _gyroScale = (2000.0f / (float)(1 << fssel)) / 32768.0f;
    _gyroFS = fssel;
}

void IMU::setAccelFS(uint8_t fssel) {
    setBank(0);
    uint8_t reg;
    readRegisters(0x50, 1, &reg);
    reg = (fssel << 5) | (reg & 0x1F);
    writeRegister(0x50, reg);
    _accelScale = (float)(1 << (4 - fssel)) / 32768.0f;
    _accelFS = fssel;
}

void IMU::calibrateGyro() {
    const uint8_t current_fssel = _gyroFS;
    setGyroFS(0x03);
    float avg[3] = {0, 0, 0};
    uint8_t buffer[14];
    int16_t raw[7];

    for (int i = 0; i < 1000; i++) {
        readAGT(buffer);
        for (int j = 0; j < 7; j++)
            raw[j] = ((int16_t)buffer[j*2] << 8) | buffer[j*2+1];

        for (int j = 0; j < 3; j++)
            avg[j] += (float)raw[j+4] / 16.4f / 1000.0f;

        HAL_Delay(1);
    }

    for (int i = 0; i < 3; i++) _gyrB[i] = avg[i];
    setGyroFS(current_fssel);
}

void IMU::calibrateAccel() {
    // Store current full-scale setting to restore later
    uint8_t currentFS = _accelFS;

    // Temporarily set to highest sensitivity for calibration
    setAccelFS(0x03); // ±2g mode, for example

    float accSum[3] = {0.0f, 0.0f, 0.0f};
    const int samples = 1000;
    uint8_t dataBuffer[14];
    int16_t rawAccel[3];

    // Collect multiple samples for averaging
    for (int i = 0; i < samples; i++) {
        readAGT(dataBuffer);
        rawAccel[0] = ((int16_t)dataBuffer[0] << 8) | dataBuffer[1];
        rawAccel[1] = ((int16_t)dataBuffer[2] << 8) | dataBuffer[3];
        rawAccel[2] = ((int16_t)dataBuffer[4] << 8) | dataBuffer[5];

        // Convert to 'g'
        float ax = (float)rawAccel[0] * _accelScale;
        float ay = (float)rawAccel[1] * _accelScale;
        float az = (float)rawAccel[2] * _accelScale;

        accSum[0] += ax;
        accSum[1] += ay;
        accSum[2] += az;

        HAL_Delay(1);
    }

    // Compute average offsets (assuming stationary on flat surface)
    float avgX = accSum[0] / samples;
    float avgY = accSum[1] / samples;
    float avgZ = accSum[2] / samples;

    // Gravity compensation: when stationary, Z should read ~+1g
    _accB[0] = avgX;
    _accB[1] = avgY;
    _accB[2] = avgZ - 1.0f;  // assuming +Z axis points upward

    // Restore original full-scale setting
    setAccelFS(currentFS);
}

void IMU::configureNotchFilter(){
	uint8_t BW_SEL = 7;
	uint32_t f_des = 1300;
	double pi = 3.14159265;
	double COSWZ = cos(2 * pi * f_des / 32);
	int NF_COSWZ = 0;
	bool NF_COSWZ_SEL = 0;
	if(abs(COSWZ) <= 0.875){
		NF_COSWZ_SEL = 0;
		NF_COSWZ = round(COSWZ * 256);
	}else{
		NF_COSWZ_SEL = 1;
		if(COSWZ > 0.875){
			NF_COSWZ = round(8 * (1 - COSWZ) * 256);
		}else{
			NF_COSWZ = round(-8 * (1 + COSWZ) * 256);
		}
	}
	setBank(1);
	writeRegister(0x0F, (uint8_t)(NF_COSWZ & 0xFF));  // Lower byte for X-axis
	writeRegister(0x10, (uint8_t)(NF_COSWZ & 0xFF));  // Lower byte for Y-axis
	writeRegister(0x11, (uint8_t)(NF_COSWZ & 0xFF));  // Lower byte for Z-axis
	writeRegister(0x12, (uint8_t)((NF_COSWZ >> 8) & 0x01));  // Upper bit for all axes

	uint8_t reg_0x12;
	readRegisters(0x12, 1, &reg_0x12);
	// Modify only necessary bits (Bit 3 = X, Bit 4 = Y, Bit 5 = Z)
	reg_0x12 &= ~(0b00111000);  // Clear bits 3, 4, 5
	reg_0x12 |= (NF_COSWZ_SEL << 3) | (NF_COSWZ_SEL << 4) | (NF_COSWZ_SEL << 5);
	writeRegister(0x12, reg_0x12);

	// Set Notch Filter Bandwidth
	writeRegister(0x13, BW_SEL << 4);


	setBank(0);
}

// Anti-alias filter config structure
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

static const AAF_Config *getAAFConfig(uint16_t bandwidth) {
    const AAF_Config *best = &aaf_table[0];
    for (size_t i = 0; i < sizeof(aaf_table)/sizeof(aaf_table[0]); i++) {
        if (aaf_table[i].bandwidth >= bandwidth) {
            best = &aaf_table[i];
            break;
        }
    }
    return best;
}

void IMU::setAntiAliasFilter(uint16_t bandwidth_hz, bool accel_enable, bool gyro_enable) {
    const AAF_Config *cfg = getAAFConfig(bandwidth_hz);

    // accel
    setBank(2);

    uint8_t reg03;
    readRegisters(0x03, 1, &reg03);
    reg03 &= ~0x7E;                         // Clear bits 6:1
    reg03 |= (cfg->delt & 0x3F) << 1;       // ACCEL_AAF_DELT
    if (!accel_enable)
        reg03 |= 1 << 0;                    // ACCEL_AAF_DIS = 1
    else
        reg03 &= ~(1 << 0);                 // Enable AAF
    writeRegister(0x03, reg03);

    writeRegister(0x04, (uint8_t)(cfg->deltsqr & 0xFF));  // Lower 8 bits of DELTSQR
    uint8_t reg05;
    readRegisters(0x05, 1, &reg05);
    reg05 &= 0x00;                          // Clear bits 7:0
    reg05 |= ((cfg->deltsqr >> 8) & 0x0F);  // Upper 4 bits of DELTSQR
    reg05 |= (cfg->bitshift << 4) & 0xF0;   // ACCEL_AAF_BITSHIFT
    writeRegister(0x05, reg05);

    // gyro
    setBank(1);

    uint8_t reg0C;
    readRegisters(0x0C, 1, &reg0C);
    reg0C &= ~0x3F;                        // Clear bits 5:0
    reg0C |= (cfg->delt & 0x3F);           // GYRO_AAF_DELT
    writeRegister(0x0C, reg0C);

    writeRegister(0x0D, (uint8_t)(cfg->deltsqr & 0xFF));  // Lower 8 bits
    uint8_t reg0E;
    readRegisters(0x0E, 1, &reg0E);
    reg0E &= 0x00;                         // Clear bits
    reg0E |= ((cfg->deltsqr >> 8) & 0x0F); // Upper 4 bits
    reg0E |= (cfg->bitshift << 4) & 0xF0;  // GYRO_AAF_BITSHIFT
    writeRegister(0x0E, reg0E);

    uint8_t reg0B;
    readRegisters(0x0B, 1, &reg0B);
    if (!gyro_enable)
        reg0B |= (1 << 1);                 // Disable Gyro AAF
    else
        reg0B &= ~(1 << 1);                // Enable Gyro AAF
    writeRegister(0x0B, reg0B);

    setBank(0);
}
*/
