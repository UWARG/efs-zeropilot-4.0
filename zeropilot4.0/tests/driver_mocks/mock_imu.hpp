#pragma once

#include <gmock/gmock.h>
#include "imu_iface.hpp"

class MockIMU : public IIMU {
public:
    MOCK_METHOD(int, init, (), (override));
    MOCK_METHOD(RawImuBatch_t, readRawData, (), (override));
    MOCK_METHOD(ScaledImuBatch_t, scaleIMUData, (const RawImuBatch_t &rawDataBatch), (override));
    MOCK_METHOD(float, getODRHz, (), (override));
    MOCK_METHOD(GyroBias_t, getGyroStartupBias, (uint8_t imuId), (override));
};
