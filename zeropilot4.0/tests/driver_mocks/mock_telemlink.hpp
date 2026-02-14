#pragma once

#include <gmock/gmock.h>
#include "telemlink_iface.hpp"

class MockTelemLink : public ITelemLink {
public:
    MOCK_METHOD(void, transmit, (const uint8_t* data, uint16_t size), (override));
    MOCK_METHOD(uint16_t, receive, (uint8_t* buffer, uint16_t bufferSize), (override));
};
