#pragma once

#include "zp_error.h"
#include <cstdint>

typedef struct AirspeedData {
    double rawPress = 0;
    double rawTemp = 0;
    double processedTemp = 0;
    double processedPress = 0;
    double airspeed = 0;
} AirspeedData_t;

enum class AsStatus_e : uint8_t {
    NORMAL  = 0b00,
    COMMAND = 0b01,
    STALE   = 0b10,
    FAULT   = 0b11
};

class IAirspeed {
protected:
    IAirspeed() = default;
public:
    virtual ~IAirspeed() = default;
    
    virtual ZP_Error getAirspeedData(AirspeedData_t* data) = 0;
};
