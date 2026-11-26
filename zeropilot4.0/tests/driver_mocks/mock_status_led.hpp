#pragma once

#include "status_led_iface.hpp"
#include <gmock/gmock.h>

class MockStatusLed : public IStatusLed {
    public:
        MOCK_METHOD(ZP_Error, setColor, (uint8_t red, uint8_t green, uint8_t blue), (override));
};
