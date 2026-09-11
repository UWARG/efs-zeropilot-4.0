#pragma once

#include "rangefinder_iface.hpp"
#include <gmock/gmock.h>

class MockRangefinder : public IRangefinder {
    public: 
        MOCK_METHOD(ZP_Error, init, (), (override));
        MOCK_METHOD(ZP_Error, readData, (RangefinderData_t &data), (override));
};
