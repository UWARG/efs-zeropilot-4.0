#pragma once
#include "power_module_iface.hpp"

class SIL_PowerModule : public IPowerModule {
private:
    PMData_t pmData = {12.0f, 5.0f, 60.0f, 0.0f, 0.0f};
    float fuelCapacity = 1.0f;
    
public:
    void set_fuel_capacity(float capacity) {
        fuelCapacity = capacity;
    }
    
    void update_from_plant(float fuel_lbs, float rpm) {
        float fuel_pct = (fuelCapacity > 0) ? (fuel_lbs / fuelCapacity * 100.0f) : 0.0f;
        pmData.busVoltage = 12.0f + (fuel_pct / 100.0f) * 2.0f;  // 12-14V
        pmData.current = rpm / 200.0f;  // Mock current from RPM
        pmData.power = pmData.busVoltage * pmData.current;
    }
    
    bool readData(PMData_t *data) override {
        *data = pmData;
        return true;
    }
};
