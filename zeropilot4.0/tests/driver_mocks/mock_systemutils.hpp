#pragma once

#include <gmock/gmock.h>
#include "systemutils_iface.hpp"

class MockSystemUtils : public ISystemUtils {
public:
    MOCK_METHOD(void, delayMs, (uint32_t delay_ms), (override));
    MOCK_METHOD(uint32_t, getCurrentTimestampMs, (), (override));
    MOCK_METHOD(void, profilerRegister, (const char* name, uint8_t* outId), (override));
    MOCK_METHOD(void, profilerBegin, (uint8_t id), (override));
    MOCK_METHOD(void, profilerEnd, (uint8_t id), (override));
    MOCK_METHOD(void, profilerGetAll, (TaskProfile* out, uint8_t* count), (override));
    MOCK_METHOD(float, dspSinf, (float x), (override));
    MOCK_METHOD(float, dspCosf, (float x), (override));
};
