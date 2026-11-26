#pragma once

#include "safety_switch_iface.hpp"
#include <gmock/gmock.h>

class MockSafetySwitch : public ISafetySwitch {
    public:
        MOCK_METHOD(bool, isSafetySwitchPressed, (), (override));
        MOCK_METHOD(void, setSafetySwitchLEDState, (bool state), (override));
        MOCK_METHOD(bool, getSafetySwitchLEDState, (), (override));
};
