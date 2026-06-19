#pragma once

#include <cstdint>
#include "zp_error.h"

typedef struct PMData {
    // Instantaneous results
    float busVoltage;
    float current;
    float power;

    // Accumulated results
    float charge;
    float energy;

} PMData_t;

class IPowerModule {
protected:
    IPowerModule() = default;

public:
    virtual ~IPowerModule() = default;
    // change value to a more explicit name (need to check function definition)
    virtual ZP_ERROR_e readData(PMData_t *data) = 0;
};