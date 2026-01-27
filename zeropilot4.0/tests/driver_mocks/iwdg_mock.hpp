#pragma once

#include <gmock/gmock.h>
#include "iwdg_iface.hpp"

class MockWatchdog : public IIndependentWatchdog {
    public:
        MOCK_METHOD(bool, refreshWatchdog, (), (override));
};



/*
MockWatchdog mockWatchdog;
EXPECT_CALL(mockWatchdog, refreshWatchdog()).WillOnce(Return(true));
bool result = mockWatchdog.refreshWatchdog();
EXPECT_TRUE(result);
*/