#pragma once

#include "magnetometer_iface.hpp"
#include <gmock/gmock.h>

class MockMagnetometer : public IMagnetometer {
    public:
        MOCK_METHOD(bool, init, (), (override));
        MOCK_METHOD(MagData_t, readData, (), (override));
};
