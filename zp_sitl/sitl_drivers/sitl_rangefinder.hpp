#pragma once

#include "rangefinder_iface.hpp"

class SITL_Rangefinder : public IRangefinder {
private: 
    RangefinderData_t data = {};
public:

    ZP_Error init() override { return ZP_ERROR_OK; }
    ZP_Error readData(RangefinderData_t &outData) override {
        outData = data;
        return ZP_ERROR_OK;
    }

    void update_from_plant(float sim_altitude) {
        data.distance = sim_altitude;
        data.signalStrength = 65535;
        data.isValid = true;
        data.isNew = true;
    }
};
