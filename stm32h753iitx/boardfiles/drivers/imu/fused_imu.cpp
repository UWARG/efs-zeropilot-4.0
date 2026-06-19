#include "fused_imu.hpp"
#include <cstring>

FusedIMU::FusedIMU(SPI_HandleTypeDef* spiHandle, IMU *imu0, IMU *imu1) : 
    spiBus(spiHandle),
    imu{imu0, imu1},
    active_imu(0){}
    
int FusedIMU::init() {
    bool status = true;
    for (int i = 0; i < NUM_IMU; i++) {
        // Init all IMUs and check IMU health from whoami
        if (imu[i]->init() != 0x47) {status = false;};
    }
    active_imu = 0;
    return status; 
}

RawImuBatch_t FusedIMU::readRawData() {
    // Check if all IMU is filled
    bool allImuFilled = true;
    for (int i = 0; i < NUM_IMU; i++) {
        if (!imuFilled[i]) {allImuFilled = false; break;}
    }
    uint16_t offset = 0;
    if (allImuFilled) {
        for (int i = 0; i < NUM_IMU; i++) {
            // Reset imuFilled
            imuFilled[i] = false;
            
            // Concatonate the batches based on imu order, sort the exact order later after scaled
            memcpy(rawFusedImuData + offset, rawImuBatch[i].data, sizeof(RawImu_t) * rawImuBatch[i].count );
            offset += rawImuBatch[i].count;
        } 
    }

    if (!imuFilled[active_imu] && imu[active_imu]->getDmaFlag()) {
        imu[active_imu]->beginRead();
    }
    
    rawFusedImuBatch.data = rawFusedImuData;
    rawFusedImuBatch.count = offset;

    return rawFusedImuBatch;
}

ScaledImuBatch_t FusedIMU::scaleIMUData(const RawImuBatch_t &rawDataBatch) {
    // Sort batches from different imus into a unified timeline
    return scaledFusedImuBatch;
}

void FusedIMU::txRxCallback() {
    imu[active_imu]->txRxCallback();
    // Switch to next imu if prev imu transfer is done (dmaDone), when done process the data
    if (imu[active_imu]->getDmaFlag()) {
        // The active IMU is finished, pass on to next IMU
        imuFilled[active_imu] = true;

        // Get data from the IMU that just finished transaction
        rawImuBatch[active_imu] = imu[active_imu]->getBatch();

        active_imu++;
        if (active_imu > (NUM_IMU - 1)) {
            active_imu = 0; // One round of data collection is finished, dont kick off another transaction
        } 
    } 
}

SPI_HandleTypeDef* FusedIMU::getSPI() {
    return spiBus;
}