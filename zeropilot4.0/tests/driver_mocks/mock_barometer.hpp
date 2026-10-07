#pragma once

#include <gmock/gmock.h>
#include "barometer_iface.hpp"

class MockBarometer : public IBarometer {
    public:
        MOCK_METHOD(ZP_Error, readData, (BaroData_t &data), (override));
};
