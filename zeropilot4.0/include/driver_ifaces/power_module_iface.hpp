#pragma once

#include <cstdint>

typedef struct {

    //instantaneous results
    float busVoltage = 0.0f;
    float current = 0.0f;
    float power = 0.0f;

    //accumulated results
    float charge = 0.0f;
    float energy = 0.0f;

} PMData_t;

class IPowerModule {
protected:
    IPowerModule() = default;

public:
    virtual ~IPowerModule() = default;
    // change value to a more explicit name (need to check function definition)
    virtual ZP_ERROR_e readData(PMData_t *data, bool *value) = 0;
};