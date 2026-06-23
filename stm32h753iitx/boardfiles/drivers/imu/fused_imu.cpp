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

            uint16_t count = rawImuBatch[i].count;
            if (count == 0) {continue;};
            
            // Concatonate the batches based on imu order, sort the exact order in scaleIMUData
            memcpy(rawFusedImuData + offset, rawImuBatch[i].data, sizeof(RawImu_t) * count );
            
            // Normalize IMU hardware timstamps to DWT ticks, cant compare hardware ticks between IMUs
            uint32_t elapsed = 0;
            uint32_t batchReadTime = rawImuBatch[i].readTime;
            uint16_t prevHwTS = rawFusedImuData[offset + (count - 1)].timestamp;
            rawFusedImuData[offset + (count - 1)].timestamp = batchReadTime;
            for (int j = count - 2; j >= 0; j--) {
                uint16_t currentHwTS = rawFusedImuData[offset + j].timestamp;
                elapsed += (uint16_t)(prevHwTS - currentHwTS); // Cast to uin16_t so if delta > 65,536(2^16, hw timstamp limit), it wraps around
                rawFusedImuData[offset + j].timestamp = batchReadTime - elapsed; // Both DWT and hw timestamp are in microseconds
                prevHwTS = currentHwTS;
            }

            offset += count;
        } 
    }
    // Start FIFO read for the active IMU if has not been started yet
    if (!imuFilled[active_imu] && imu[active_imu]->getDmaFlag()) {
        imu[active_imu]->beginRead();
    }
    
    rawFusedImuBatch.data = rawFusedImuData;
    rawFusedImuBatch.count = offset;

    return rawFusedImuBatch;
}

ScaledImuBatch_t FusedIMU::scaleIMUData(const RawImuBatch_t &rawDataBatch) {
    // Scale IMU data
    RawImuBatch_t temp; 
    uint16_t offset = 0;
    for (int i = 0; i < NUM_IMU; i++) {
        uint16_t count = rawImuBatch[i].count;
        if (count == 0) {
            scaledImuBatch[i].count = 0;
            continue;
        }
        
        temp.data = rawDataBatch.data + offset;
        temp.count = count;
        scaledImuBatch[i] = imu[i]->scaleIMUData(temp);
        offset += count;
    }

    // Sort batches from different imus into a unified timeline
    int i = 0;
    int j = 0;
    int k = 0;
    while (i < scaledImuBatch[0].count && j < scaledImuBatch[1].count) {
        if (scaledImuBatch[0].data[i].timestamp < scaledImuBatch[1].data[j].timestamp) {
            scaledFusedImuData[k] = scaledImuBatch[0].data[i];
            i++;
        } else {
            scaledFusedImuData[k] = scaledImuBatch[1].data[j];
            j++;
        }
        k++;
    }
    while (i < scaledImuBatch[0].count) {
        scaledFusedImuData[k] = scaledImuBatch[0].data[i];
        k++;
        i++;
    }
    while (j < scaledImuBatch[1].count) {
        scaledFusedImuData[k] = scaledImuBatch[1].data[j];
        k++;
        j++;
    }
    
    scaledFusedImuBatch.data = scaledFusedImuData;
    scaledFusedImuBatch.count = k;
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