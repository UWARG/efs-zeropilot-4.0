#pragma once

#include <cstdint>

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

    virtual bool readData(PMData_t *data) = 0;
};