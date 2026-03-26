#pragma once

#include <cstdint>

typedef struct PMData {

    //instantaneous results
    float busVoltage;
    float current;
    float power;

    //accumulated results
    float charge;
    float energy;

} PMData_t;

class IPowerModule {
protected:
    IPowerModule() = default;

public:
    virtual ~IPowerModule() = default;
    // change value to a more explicit name (need to check function definition)
    virtual ZP_ERROR_e readData(PMData_t *data, bool *value) = 0;
};