#pragma once

#include "magnetometer_iface.hpp"
#include <gmock/gmock.h>

class MockMagnetometer : public IMagnetometer {
    public:
        MOCK_METHOD(bool, init, (), (override));
        MOCK_METHOD(MagData_t, readData, (), (override));
        MOCK_METHOD(void, startCalibration, (), (override));
        MOCK_METHOD(void, cancelCalibration, (), (override));
        MOCK_METHOD(MagCalStatus_t, getCalibrationStatus, (), (override));
        MOCK_METHOD(const MagCalConstants_t &, getCalibrationConstants, (), (const, override));
};
