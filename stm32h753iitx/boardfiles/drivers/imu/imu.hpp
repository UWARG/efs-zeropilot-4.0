// IMU.hpp

#ifndef IMU_HPP
#define IMU_HPP

#include "imu_iface.hpp"
#include "stm32h7xx_hal.h"
#include <cstdint>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include "imu_datatypes.hpp"

class IMU : public IIMU
{
private:
	SPI_HandleTypeDef *_spi;
	GPIO_TypeDef *_csPort;
	uint16_t _csPin;

	static constexpr float GYRO_SEN_SCALE_FACTOR = 16.4f;			 // determined by GYRO_FS_SEL, page 11
	static constexpr float ACCEL_SEN_SCALE_FACTOR = 2048.0f / 9.81f; // determined by ACCEL_FS_SEL, page 12, scale to m/s^2

	static constexpr uint8_t PACKET_SIZE = 16;
	static constexpr uint8_t MAX_PACKETS = 128;
	static constexpr uint16_t RX_BUFFER_SIZE = MAX_PACKETS * PACKET_SIZE + 1; // each packet size is 14 bytes
	volatile uint8_t imuTxBuffer[RX_BUFFER_SIZE];							  // only first bit register addr to read sensor data, rest 0
	volatile uint8_t imuRxBuffer[RX_BUFFER_SIZE];							  // first byte is dummy, next 14 bytes are data received

	uint8_t currRegisterBank = 5; // invalid initial state
	// volatile uint8_t spi_tx_rxFlag = 1; // set to 1 to initiate first read
	typedef enum
	{
		COUNT,
		DATA
	} RxStates_e;
	volatile RxStates_e rxFlag = COUNT;
	volatile bool dmaDone = true;

	RawImu_t rawData[MAX_PACKETS] = {};
	RawImuBatch_t rawImuDataBatch = {};
	ScaledImu_t scaledData[MAX_PACKETS] = {};
	ScaledImuBatch_t scaledImuDataBatch = {};

	uint16_t fifoSize = 0;

	// Utility functions
	HAL_StatusTypeDef writeRegister(uint8_t bank, uint8_t register_addr, uint8_t data); // blocking
	HAL_StatusTypeDef readRegister(uint8_t bank, uint8_t register_addr, uint8_t *data); // blocking

	void csLow();
	void csHigh();
	HAL_StatusTypeDef setBank(uint8_t bank);
	void reset();
	uint8_t whoAmI();
	void flushFIFO();
	void processRawData();
	void dmaTransfer();
	

	// Configuration
	void setLowNoiseMode();
	void setFIFO();

	// Filtering
	float lowPassFilter(float raw_value, int select);

	// Internal variables
	float _alpha;
	float _filteredGyro[3];

	// TODO: below code needs to be tested and verified

	/*
	// Calibration
	void calibrateGyro();
	void calibrateAccel();

	// Configuration
	void setAccelFS(uint8_t fssel);
	void setGyroFS(uint8_t fssel);

	// Filtering
	void configureNotchFilter();
	void setAntiAliasFilter(uint16_t bandwidth_hz, bool accel_enable, bool gyro_enable);

	// Internal variables
	float _gyroScale;
	float _accelScale;
	uint8_t _gyroFS;
	uint8_t _accelFS;
	float _gyrB[3]; // currently not used to correct readings
	float _accB[3]; // currently not used to correct readings
	*/

public:
	IMU(SPI_HandleTypeDef *spiHandle, GPIO_TypeDef *csPort, uint16_t csPin);

	// Initialization
	int init() override;

	// Data reading
	// First read returns all 0s, subsequent reads return latest data
	RawImuBatch_t readRawData() override; // non-blocking

	ScaledImuBatch_t scaleIMUData(const RawImuBatch_t &rawDataBatch) override;

	// put this in void HAL_SPI_TxRxCpltCallback (SPI_HandleTypeDef * hspi)
	void txRxCallback();

	SPI_HandleTypeDef *getSPI();
};

#endif
