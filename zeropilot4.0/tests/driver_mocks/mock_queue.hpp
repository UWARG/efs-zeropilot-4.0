#pragma once

#include <gmock/gmock.h>
#include "queue_iface.hpp"

template <typename T>
class MockMessageQueue : public IMessageQueue<T> {
    public:
        MOCK_METHOD(int, get, (T *message), (override));
        MOCK_METHOD(int, push, (T *message), (override));
        MOCK_METHOD(int, count, (), (override));
        MOCK_METHOD(int, remainingCapacity, (), (override));
};
