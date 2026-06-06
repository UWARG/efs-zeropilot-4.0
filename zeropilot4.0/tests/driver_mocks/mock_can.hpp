#pragma once

#include <gmock/gmock.h>
#include "can_iface.hpp"

class MockCAN : public ICAN {
public:
    MOCK_METHOD(bool, routineTasks, (), (override));
};
