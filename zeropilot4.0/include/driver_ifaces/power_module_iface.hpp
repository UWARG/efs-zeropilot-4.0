#pragma once

#include "power_module_datatypes.hpp"

class IPowerModule {
protected:
    IPowerModule() = default;

public:
    virtual ~IPowerModule() = default;

    virtual bool readData(PMData_t *data) = 0;
};