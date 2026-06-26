#pragma once

#include "imu_iface.hpp"
#include "imu.hpp"

class FusedIMU : public IIMU {
    public:
        FusedIMU(SPI_HandleTypeDef* spiHandle, IMU *imu0, IMU *imu1);

        int init() override;
        
        RawImuBatch_t readRawData() override;

        ScaledImuBatch_t scaleIMUData(const RawImuBatch_t &rawDataBatch) override;
        
        // Called in HAL_SPI_TxRxCpltCallback
        void txRxCallback();

        SPI_HandleTypeDef *getSPI();

    private:
        SPI_HandleTypeDef *spiBus;

        static constexpr uint8_t NUM_IMU = 2;
        static constexpr uint16_t MAX_FUSED_PACKET_SIZE = IMU::MAX_PACKETS * NUM_IMU;

        IMU *imu[NUM_IMU] = {};

        volatile uint8_t active_imu;

        RawImuBatch_t rawImuBatch[NUM_IMU] = {};
        ScaledImuBatch_t scaledImuBatch[NUM_IMU] = {};

        RawImu_t rawFusedImuData[MAX_FUSED_PACKET_SIZE] = {};
		RawImuBatch_t rawFusedImuBatch = {};
		ScaledImu_t scaledFusedImuData[MAX_FUSED_PACKET_SIZE] = {};
        ScaledImuBatch_t scaledFusedImuBatch = {};

        volatile bool imuFilled[NUM_IMU] = {};

        uint32_t lastEmittedTimestamp = 0;
        bool haveEmitted = false;

};