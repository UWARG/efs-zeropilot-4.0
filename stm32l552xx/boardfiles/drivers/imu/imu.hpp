// IMU.hpp
#pragma once

#include "imu_iface.hpp"
#include "stm32l5xx_hal.h"
#include <cstdint>
#include "imu_datatypes.hpp"

typedef enum : uint8_t {
	IMU_ODR_32KHZ = 0b0001,
	IMU_ODR_16KHZ = 0b0010,
	IMU_ODR_8KHZ = 0b0011,
	IMU_ODR_4KHZ = 0b0100,
	IMU_ODR_2KHZ = 0b0101,
	IMU_ODR_1KHZ = 0b0110,
	IMU_ODR_500HZ = 0b1111,
	IMU_ODR_200HZ = 0b0111,
	IMU_ODR_100HZ = 0b1000,
	IMU_ODR_50HZ = 0b1001,
	IMU_ODR_25HZ = 0b1010,
	IMU_ODR_12HZ5 = 0b1011
} ImuOdrConfig_t;

class IMU : public IIMU {
	public:
		IMU(SPI_HandleTypeDef *spiHandle, GPIO_TypeDef *csPort, uint16_t csPin, uint8_t imuId, ImuOdrConfig_t odrConfig);
	
		// Initialization
		int init() override;
	
		// Data reading, first read returns all 0s, subsequent reads return latest data
		RawImuBatch_t readRawData() override; // non-blocking
		ScaledImuBatch_t scaleIMUData(const RawImuBatch_t &rawDataBatch) override;
	
		void txRxCallback(); // Called in HAL_SPI_TxRxCpltCallback
	
		SPI_HandleTypeDef *getSPI();
		bool getDmaFlag();

		void beginRead();
		RawImuBatch_t getBatch();
		float getODRHz() override;
		GyroStartupBias_t getGyroStartupBias() override;
		
		static constexpr float GYRO_SEN_SCALE_FACTOR = 16.4f;			 // Determined by GYRO_FS_SEL, page 11
		static constexpr float ACCEL_SEN_SCALE_FACTOR = 2048.0f / 9.81f; // Determined by ACCEL_FS_SEL, page 12, scale to m/s^2
		static constexpr uint8_t MAX_PACKETS = 128; // User defined max packet reads per batch, has to be <= FIFO_HW_MAX_PACKETS
		
	private:
		SPI_HandleTypeDef *spi;
		GPIO_TypeDef *csPort;
		uint16_t csPin;
		const uint8_t imuId;
		const ImuOdrConfig_t imuOdr;
		
		static constexpr uint8_t PACKET_SIZE = 16;
		static constexpr uint8_t FIFO_HW_MAX_PACKETS = 128; // Hardware FIFO packet limit
		static constexpr uint16_t RX_BUFFER_SIZE = MAX_PACKETS * PACKET_SIZE + 1;

		typedef enum
		{
			COUNT,
			DATA
		} RxStates_e;

		volatile uint8_t imuTxBuffer[RX_BUFFER_SIZE]; // First bit should be 1 for register read
		volatile uint8_t imuRxBuffer[RX_BUFFER_SIZE]; // First byte is dummy, rest are data received
		volatile RxStates_e rxFlag = COUNT;
		volatile bool dmaDone = true; // True so can kick off first transfer
		uint8_t currRegisterBank = 5; // Invalid initial state
		uint16_t fifoSize = 0;

		RawImu_t rawData[MAX_PACKETS] = {};
		RawImuBatch_t rawImuDataBatch = {};
		ScaledImu_t scaledData[MAX_PACKETS] = {};
		ScaledImuBatch_t scaledImuDataBatch = {};
		GyroStartupBias_t gyroBias = {};

		// Utility functions, blocking
		HAL_StatusTypeDef writeRegister(uint8_t bank, uint8_t registerAddr, uint8_t data); 
		HAL_StatusTypeDef readRegister(uint8_t bank, uint8_t registerAddr, uint8_t* data, uint8_t length); 
		HAL_StatusTypeDef setBank(uint8_t bank);

		void csLow();
		void csHigh();
		void reset();
		uint8_t whoAmI();
		void flushFIFO();
		void dmaTransfer();
		
		// Configuration
		void setLowNoiseMode();
		void setFIFO();
		void setODR();
		
		// Processing and filtering
		void processRawData();
		float lowPassFilter(float rawValue, int select);

		// Internal variables
		float alpha;
		float filteredGyro[3];
};
