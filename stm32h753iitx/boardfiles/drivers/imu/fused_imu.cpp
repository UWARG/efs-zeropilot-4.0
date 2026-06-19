#include "fused_imu.hpp"

FusedIMU::FusedIMU(IMU *imu0, IMU *imu1) : 
    imu{imu0, imu1},
    active_imu(0){}
    
int FusedIMU::init() {
    // check imu health from whoami
    for (int i = 0; i < NUM_IMU; i++) {
        imu[i]->init();
    }
    active_imu = 0;
    imu[active_imu]->beginRead();
    return; // return address/health?
}

RawImuBatch_t FusedIMU::readRawData() {
    imu[active_imu]->readRawData();
}

void FusedIMU::txRxCallback() {
    // switch to next imu if prev imu transfer is done (dmaDone), when done process the data
    
}