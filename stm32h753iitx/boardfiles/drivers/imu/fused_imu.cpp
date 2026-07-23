#include "fused_imu.hpp"
#include <cstring>

FusedIMU::FusedIMU(SPI_HandleTypeDef* spiHandle, IMU *imu0, IMU *imu1) : 
    spiBus(spiHandle),
    imu{imu0, imu1},
    active_imu(0) {}
    
int FusedIMU::init() {
    bool status = true;
    for (int i = 0; i < NUM_IMU; i++) {
        // Init all IMUs and check IMU health from whoami
        if (imu[i]->init() == -1) {
            status = false;
        }
    }
    active_imu = 0;
    return status ? 0 : -1;
}

RawImuBatch_t FusedIMU::readRawData() {
    // Check if all IMU is filled
    bool allImuFilled = true;
    for (int i = 0; i < NUM_IMU; i++) {
        if (!imuFilled[i]) {
            allImuFilled = false;
            break;
        }
    }
    uint16_t offset = 0;
    if (allImuFilled) {
        for (int i = 0; i < NUM_IMU; i++) {
            imuFilled[i] = false;

            uint16_t count = rawImuBatch[i].count;
            if (count == 0) continue; // No data from this IMU, skip
            
            // Concatonate the batches based on imu order, sort the exact order later in scaleIMUData
            memcpy(rawFusedImuData + offset, rawImuBatch[i].data, sizeof(RawImu_t) * count);
            
            // Normalize IMU hardware timstamps to DWT ticks, cant compare hardware ticks between IMUs
            uint32_t elapsed = 0;
            uint32_t batchReadTime = rawImuBatch[i].readTime;
            uint16_t prevHwTS = rawFusedImuData[offset + (count - 1)].timestamp;
            rawFusedImuData[offset + (count - 1)].timestamp = batchReadTime;
            for (int j = count - 2; j >= 0; j--) {
                uint16_t currentHwTS = rawFusedImuData[offset + j].timestamp;
                elapsed += (uint16_t)(prevHwTS - currentHwTS); // Cast to uint16_t so if delta > 65,536(2^16, hw timstamp limit) it wraps around
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
    // Guard to prevent recalculations when IMUs not filled with new data
    if (rawDataBatch.count == 0) {
        scaledFusedImuBatch.count = 0;
        return scaledFusedImuBatch;
    }
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

    uint16_t idx[NUM_IMU] = {}; // idx[i] = how far consumed IMU i is
    uint16_t k = 0; // Index for scaledFusedImuData
    while (true) {
        int smallestIMU = -1;
        uint32_t smallestTimestamp = 0;
        for (int i = 0; i < NUM_IMU; i++) {
            if (idx[i] >= scaledImuBatch[i].count) continue; // This IMU is all merged

            uint32_t timestamp = scaledImuBatch[i].data[idx[i]].timestamp;
            if (smallestIMU == -1 || (int32_t)(timestamp - smallestTimestamp) < 0) { // First iteration or a smaller one
                smallestIMU = i;
                smallestTimestamp = timestamp;
            }
        }

        if (smallestIMU == -1) break; // All IMU has been merged, done

        scaledFusedImuData[k] = scaledImuBatch[smallestIMU].data[idx[smallestIMU]];
        k++;
        idx[smallestIMU]++;
    }

    // Disregard the data that are earlier than the last data consumed from last call
    int start = 0;
    if (haveEmitted) {
        // merged array is sorted, so the stale samples are a contiguous prefix
        while (start < k && (int32_t)(scaledFusedImuData[start].timestamp - lastEmittedTimestamp) <= 0) {
            start++;   // drop: already covered by a previous round
        }
    }
    if (k > start) {
        lastEmittedTimestamp = scaledFusedImuData[k - 1].timestamp;
        haveEmitted = true;
    }
    
    scaledFusedImuBatch.data = scaledFusedImuData + start;
    scaledFusedImuBatch.count = k - start;
    return scaledFusedImuBatch;
}

void FusedIMU::txRxCallback() {
    imu[active_imu]->txRxCallback();
    // Switch to next imu if prev imu transfer is done (dmaDone), when done process the data
    if (imu[active_imu]->getDmaFlag()) {
        // Get data from the IMU that just finished transaction
        rawImuBatch[active_imu] = imu[active_imu]->getBatch();

        // The active IMU is finished, pass on to next IMU
        imuFilled[active_imu] = true;

        active_imu++;
        if (active_imu > (NUM_IMU - 1)) {
            active_imu = 0; // One round of data collection is finished, dont kick off another transaction
        } else {
            imu[active_imu]->beginRead();
        }
    } 
}

SPI_HandleTypeDef* FusedIMU::getSPI() {
    return spiBus;
}

float FusedIMU::getODRHz() {
    return imu[0]->getODRHz();
}

GyroStartupBias_t FusedIMU::getGyroStartupBias(uint8_t imuId) {
    return imu[imuId]->getGyroStartupBias(imuId);
}
