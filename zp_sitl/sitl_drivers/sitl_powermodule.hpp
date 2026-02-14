#pragma once
#include "power_module_iface.hpp"
#include "sitl_driver_configs.hpp"
#include <algorithm>

class SITL_PowerModule : public IPowerModule {
private:
    using Config = SITL_Driver_Configs::SITL_PowerModule_Config;

    // Initialized for 4S nominal voltage (~14.8V)
    PMData_t pmData = {14.8f, 0.0f, 0.0f, 0.0f, 100.0f};
    float maxCapacity = 1.0f;

public:
    void set_max_batt_capacity(float capacity) {
        if (capacity > 0) maxCapacity = capacity;
    }
    
    void update_from_plant(float remainingCapacity, float rpm) {
        float capacityRatio = (maxCapacity > 0) ? (remainingCapacity / maxCapacity) : 0.0f;
        capacityRatio = std::clamp(capacityRatio, 0.0f, 1.0f);

        // Linear interpolation for 4S Voltage
        pmData.busVoltage = Config::V_EMPTY + (capacityRatio * (Config::V_FULL - Config::V_EMPTY));
        
        // Dynamic stats
        pmData.current = rpm / 100.0f + 3.0f; // Simulate current draw based on RPM
        pmData.power = pmData.busVoltage * pmData.current;
        pmData.charge = capacityRatio * Config::MAX_BATTERY_CAPACITY_AH;
        pmData.energy = pmData.charge * Config::V_NOMINAL;
    }
    
    bool readData(PMData_t *data) override {
        if (!data) return false;
        *data = pmData;
        return true;
    }
};
