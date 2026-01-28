#pragma once

#include <gmock/gmock.h>
#include "logger_iface.hpp"

class MockLogger : public ILogger {
    MOCK_METHOD(int, log, (char message[100]), (override));
    MOCK_METHOD(int, log, (char messages[][100], int count), (override));
    MOCK_METHOD(int, init, (), (override));
};