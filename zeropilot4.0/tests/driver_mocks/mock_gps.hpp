#pragma once 

#include "gps_iface.hpp"
#include <gmock/gmock.h>


class MockGPS : public IGPS {
    public:
        MOCK_METHOD(GpsData_t, readData, (), (override));
}; 