#pragma once

#include "rangefinder_iface.hpp"
#include <gmock/gmock.h>

class MockRangefinder : public IRangefinder {
    public: 
        MOCK_METHOD(RangefinderData_t, readData, (), (override));
}