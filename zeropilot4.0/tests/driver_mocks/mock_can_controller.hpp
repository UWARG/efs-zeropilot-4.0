#pragma once

#include <gmock/gmock.h>
#include "can_controller_iface.hpp"

class MockCANController : public ICANController {
public:
    MOCK_METHOD(bool, routineTasks, (), (override));
};
