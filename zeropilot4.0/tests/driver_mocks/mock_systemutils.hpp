#pragma once

#include <gmock/gmock.h>
#include "systemutils_iface.hpp"

class MockSystemUtils : public ISystemUtils {
public:
    MOCK_METHOD(void, delayMs, (uint32_t delay_ms), (override));
    MOCK_METHOD(uint32_t, getCurrentTimestampMs, (), (override));
};
