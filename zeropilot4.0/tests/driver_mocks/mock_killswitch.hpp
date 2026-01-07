#pragma once

#include "driver_ifaces/killswitch_iface.hpp"
#include <gmock/gmock.h>

class MockKillSwitch : public IKillSwitch {
public:
    MOCK_METHOD(bool, isPressed, (), (const, override));
    MOCK_METHOD(void, forceKill, (), (override));
    MOCK_METHOD(void, clearKill, (), (override));
    MOCK_METHOD(void, buzzerOn, (), (override));
    MOCK_METHOD(void, buzzerOff, (), (override));
};

