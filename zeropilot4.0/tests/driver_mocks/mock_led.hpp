#pragma once

#include <gmock/gmock.h>
#include "led_iface.hpp"

class MockLed : public ILed {
    public:
        MOCK_METHOD(void , ledOn, (), (override));
        MOCK_METHOD(void, ledOff, (), (override));
        MOCK_METHOD(void, init, (), (override));
};
