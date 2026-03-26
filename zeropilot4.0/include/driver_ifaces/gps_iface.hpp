#pragma once

#include "gps_datatypes.hpp"
#include "error.h"

class IGPS {
protected:
    IGPS() = default;

public:
    virtual ~IGPS() = default;

    virtual ZP_ERROR_e readData(GpsData_t *data) = 0;
};
