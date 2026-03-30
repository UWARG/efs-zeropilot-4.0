#pragma once

#include <gmock/gmock.h>
#include "safety_switch_iface.hpp"

class MockSafetySwitch : public ISafetySwitch {
    public:
        MOCK_METHOD(bool, isOn, (), (override));
};
