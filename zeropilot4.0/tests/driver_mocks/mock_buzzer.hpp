#pragma once

#include <gmock/gmock.h>
#include "buzzer_iface.hpp"

class MockBuzzer : public IBuzzer {
    public:
        MOCK_METHOD(void, buzzerOn, (), (override));
        MOCK_METHOD(void, buzzerOff, (), (override));
        MOCK_METHOD(void, init, (), (override));
};
