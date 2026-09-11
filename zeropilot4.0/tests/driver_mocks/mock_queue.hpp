#pragma once

#include <gmock/gmock.h>
#include "queue_iface.hpp"

template <typename T>
class MockMessageQueue : public IMessageQueue<T> {
    public:
        MOCK_METHOD(ZP_Error, get, (T *message), (override));
        MOCK_METHOD(ZP_Error, push, (T *message), (override));
        MOCK_METHOD(ZP_Error, count, (int &count_value), (override));
        MOCK_METHOD(ZP_Error, remainingCapacity, (int &capacity), (override));
};
