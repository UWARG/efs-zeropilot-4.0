#pragma once

#include "rangefinder_iface.hpp"

class SITL_Rangefinder : public IRangefinder {
private: 
    RangefinderData_t data = {};
public:

    int init() override { return 0; }
    RangefinderData_t readData() override { return data; }

    void update_from_plant(float sim_altitude) {
        data.distance = sim_altitude;
        data.isValid = true;
        data.isNew = true;
    }
};