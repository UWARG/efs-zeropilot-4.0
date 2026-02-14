#pragma once

#include <gmock/gmock.h>
#include "logger_iface.hpp"

class MockLogger : public ILogger {
public:
    MOCK_METHOD(int, log, (const char message[100]), (override));
    MOCK_METHOD(int, log, (const char messages[][100], int count), (override));
};
