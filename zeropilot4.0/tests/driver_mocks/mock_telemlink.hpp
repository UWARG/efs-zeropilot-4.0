#pragma once

#include <gmock/gmock.h>
#include "telemlink_iface.hpp"

class MockTelemLink : public ITelemLink {
public:
    MOCK_METHOD(ZP_Error, transmit, (const uint8_t* data, uint16_t size), (override));
    MOCK_METHOD(ZP_Error, receive, (uint8_t* buffer, uint16_t bufferSize, uint16_t &received_size), (override));
};
