// IMU.cpp

#include "imu.hpp"
#include <string.h>


#define REG_BANK_SEL          0x76
#define UB0_REG_WHO_AM_I      0x75
#define UB0_REG_DEVICE_CONFIG 0x11
#define UB0_REG_PWR_MGMT0     0x4E
#define UB0_REG_TEMP_DATA1    0x1D


IMU::IMU(SPI_HandleTypeDef* spiHandle, GPIO_TypeDef* csPort, uint16_t csPin)
    : _spi(spiHandle), _csPort(csPort), _csPin(csPin),
      _alpha(0.1f)
{
    _filteredGyro[0] = _filteredGyro[1] = _filteredGyro[2] = 0.0f;

    memset((void*)imu_tx_buffer, 0, RX_BUFFER_SIZE);
    memset((void*)imu_rx_buffer, 0, RX_BUFFER_SIZE);

    // only setting first bit, rest should be 0
    imu_tx_buffer[0] = UB0_REG_TEMP_DATA1 | 0b10000000; // set 8-th bit to 1 for read, page 53
}


void IMU::csLow() {
    HAL_GPIO_WritePin(_csPort, _csPin, GPIO_PIN_RESET);
}


void IMU::csHigh() {
    HAL_GPIO_WritePin(_csPort, _csPin, GPIO_PIN_SET);
}


HAL_StatusTypeDef IMU::setBank(uint8_t bank) {
    if (curr_register_bank == bank) {
        return HAL_OK;
    }
    uint8_t tx_buf[2] = {REG_BANK_SEL, bank};
    csLow();
    HAL_StatusTypeDef status = HAL_SPI_Transmit(_spi, tx_buf, 2, HAL_MAX_DELAY);
    csHigh();

    curr_register_bank = bank;
    return status;
}


HAL_StatusTypeDef IMU::readRegister(uint8_t bank, uint8_t register_addr, uint8_t* data) {
    
    HAL_StatusTypeDef status = setBank(bank);
    if (status != HAL_OK) {
        return status;
    }
    
    uint8_t tx[2] = {(uint8_t)(register_addr | 0b10000000), 0}; // set 8-th bit to 1 for read, page 53
    uint8_t rx[2] = {0, 0};

    csLow();
    status = HAL_SPI_TransmitReceive(_spi, tx, rx, 2, HAL_MAX_DELAY);
    csHigh();

    *data = rx[1];

    return status;
}


HAL_StatusTypeDef IMU::writeRegister(uint8_t bank, uint8_t register_addr, uint8_t data) {
    
    HAL_StatusTypeDef status = setBank(bank);
    if (status != HAL_OK) {
        return status;
    }
    uint8_t tx_buf[2] = {register_addr, data};
    csLow();
    status = HAL_SPI_Transmit(_spi, tx_buf, 2, HAL_MAX_DELAY);
    csHigh();
    return status;
}


RawImu_t IMU::readRawData() {
    setBank(0);

    if (spi_tx_rx_flag) {
        spi_tx_rx_flag = 0;

        processRawData();

        csLow();
        HAL_SPI_TransmitReceive_DMA(_spi, (uint8_t*)imu_tx_buffer, (uint8_t*)imu_rx_buffer, RX_BUFFER_SIZE);
    }

    return raw_imu_data;
}


void IMU::setLowNoiseMode() {
    writeRegister(0, UB0_REG_PWR_MGMT0, 0x0F);
}


void IMU::reset() {
    setBank(0);
    writeRegister(0, UB0_REG_DEVICE_CONFIG, 0x01);
    HAL_Delay(1); // need one ms delay after reset, 
}


uint8_t IMU::whoAmI() {
    uint8_t buffer;
    readRegister(0, UB0_REG_WHO_AM_I, &buffer);
    return buffer;
}


float IMU::lowPassFilter(float raw_value, int select) {
    _filteredGyro[select] = _alpha * raw_value + (1 - _alpha) * _filteredGyro[select];
    return _filteredGyro[select];
}


int IMU::init() {
    csHigh();
    reset();
    uint8_t address = whoAmI();
    setLowNoiseMode();
    // TODO: enable and test below configurations
    // setAccelFS(0b01101001);
    // configureNotchFilter();
	// setAntiAliasFilter(213, true, true);
    // calibrateGyro();
    return address;
}

void IMU::txRxCallback() {
    csHigh();
    spi_tx_rx_flag = 1;
}

void IMU::processRawData() {
    int16_t raw[7];

    for (int i = 0; i < 7; i++)
        raw[i] = ((int16_t)imu_rx_buffer[i*2+1] << 8) | imu_rx_buffer[i*2+2];

    // NED
    raw_imu_data.xacc = raw[1];
    raw_imu_data.yacc = raw[2];
    raw_imu_data.zacc = raw[3];
    raw_imu_data.xgyro = -raw[4];
    raw_imu_data.ygyro = -raw[5];
    raw_imu_data.zgyro = -raw[6];


    // float acc_temp[3];
    // float gyr_temp[3];

    // acc_temp[0] = (float)raw[1] / 2048.0f * 9.81f;
    // acc_temp[1] = (float)raw[2] / 2048.0f * 9.81f;
    // acc_temp[2] = (float)raw[3] / 2048.0f * 9.81f;

    // gyr_temp[0] = lowPassFilter((float)raw[4] / 16.4f, 0);
    // gyr_temp[1] = lowPassFilter((float)raw[5] / 16.4f, 1);
    // gyr_temp[2] = lowPassFilter((float)raw[6] / 16.4f, 2);

    // // NED
    // raw_imu_data.xacc = (float)acc_temp[1];
    // raw_imu_data.yacc = (float)acc_temp[0];
    // raw_imu_data.zacc = ((float)acc_temp[2]);
    // raw_imu_data.xgyro = ((float)-gyr_temp[1]);
    // raw_imu_data.ygyro = ((float)-gyr_temp[0]);
    // raw_imu_data.zgyro = ((float)-gyr_temp[2]);
}

ScaledImu_t IMU::scaleIMUData(const RawImu_t &rawData) {
    ScaledImu_t scaledData;

    scaledData.xacc = (float)rawData.xacc / ACCEL_SEN_SCALE_FACTOR;
    scaledData.yacc = (float)rawData.yacc / ACCEL_SEN_SCALE_FACTOR;
    scaledData.zacc = (float)rawData.zacc / ACCEL_SEN_SCALE_FACTOR;
    scaledData.xgyro = lowPassFilter((float)rawData.xgyro / GYRO_SEN_SCALE_FACTOR, 0);
    scaledData.ygyro = lowPassFilter((float)rawData.ygyro / GYRO_SEN_SCALE_FACTOR, 1);
    scaledData.zgyro = lowPassFilter((float)rawData.zgyro / GYRO_SEN_SCALE_FACTOR, 2);

    return scaledData;
}

SPI_HandleTypeDef* IMU::getSPI() {
    return _spi;
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
