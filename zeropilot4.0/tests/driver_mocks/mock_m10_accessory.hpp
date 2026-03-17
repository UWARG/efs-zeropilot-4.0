#pragma once

#include <gmock/gmock.h>
#include "m10_accessory_iface.hpp"

class MockM10Accessory : public IM10Accessory {
    public:
        MOCK_METHOD(bool, readSafetySwitch, (), (override));
        MOCK_METHOD(void, buzzerOn, (), (override));
        MOCK_METHOD(void, buzzerOff, (), (override));
};
