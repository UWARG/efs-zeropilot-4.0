#pragma once

#include "imu_iface.hpp"
#include "imu.hpp"

class FusedIMU : public IIMU {
    public:
        FusedIMU(IMU *imu0, IMU *imu1);

        int init() override;
        
        RawImuBatch_t readRawData() override;

        ScaledImuBatch_t scaleIMUData(const RawImuBatch_t &rawDataBatch) override;
        
        // Called in HAL_SPI_TxRxCpltCallback
        void txRxCallback();

    private:
        static constexpr uint8_t NUM_IMU = 2;
        IMU *imu[NUM_IMU] = {};

        uint8_t active_imu;

        RawImuBatch_t fusedRawImuBatch;
        ScaledImuBatch_t fusedScaledImuBatch;

        bool imuHealth[NUM_IMU];

};