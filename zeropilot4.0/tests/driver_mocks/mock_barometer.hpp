#pragma once

#include <gmock/gmock.h>
#include "barometer_iface.hpp"

class MockBarometer : public IBarometer {
    public:
        MOCK_METHOD(bool, readData, (BaroData_t *data), (override));
};