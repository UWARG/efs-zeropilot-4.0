#pragma once
#include <gmock/gmock.h>
#include "airspeed_iface.hpp"

class MockAirspeed : public IAirspeed {
    public:
        MOCK_METHOD(ZP_Error, getAirspeedData, (AirspeedData_t* data), (override));
};
