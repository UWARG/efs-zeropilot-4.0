#pragma once
#include "airspeed_iface.hpp"
#include "sitl_driver_configs.hpp"

class SITL_Airspeed : public IAirspeed {
private:
    double airspeedData = 0.0f;
    
public:
    void update_from_plant(double airspeed_mps) {
        airspeedData = airspeed_mps;
    }
    
    bool getAirspeedData(double* data_out) override {
        *data_out = airspeedData;
        return true;
    }
};
