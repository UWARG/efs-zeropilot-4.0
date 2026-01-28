#pragma once

#include "gps_datatypes.hpp"

class IGPS {
protected:
    IGPS() = default;

public:
    virtual ~IGPS() = default;

    virtual GpsData_t readData() = 0;
};
