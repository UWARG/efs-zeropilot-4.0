#pragma once

#include <gmock/gmock.h>
#include "imu_iface.hpp"

class MockIMU : public IIMU {
public:
    MOCK_METHOD(int, init, (), (override));
    MOCK_METHOD(RawImu_t, readRawData, (), (override));
    MOCK_METHOD(ScaledImu_t, scaleIMUData, (const RawImu_t &rawData), (override));
};
