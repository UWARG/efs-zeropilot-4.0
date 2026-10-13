#pragma once

#include "magnetometer_iface.hpp"
#include <gmock/gmock.h>

class MockMagnetometer : public IMagnetometer {
    public:
        MOCK_METHOD(ZP_Error, init, (), (override));
        MOCK_METHOD(ZP_Error, readData, (MagData_t &data), (override));
};
