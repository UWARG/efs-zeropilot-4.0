#pragma once

#include "can_listener_iface.hpp"
#include <gmock/gmock.h>


class MockCanListener : public ICanListener {
    public:
        MOCK_METHOD(ZP_Error, onTransfer, (CanardRxTransfer *transfer), (override));
};
