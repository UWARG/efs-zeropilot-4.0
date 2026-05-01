#pragma once
#include "barometer_iface.hpp"
#include "sitl_driver_configs.hpp"

class SITL_Barometer : public IBarometer {
    private: 
        using Config = SITL_Driver_Configs::SITL_Barometer_Config;

        // Initialized for 0 everything
        BaroData_t baroData = {0.0f, 0.0f, 0.0f};
        float maxCapacity = 1.0f;
    public:
        bool readData(BaroData_t *data) override {
            if (!data) return false;
            *data = baroData;
            return true;
        }
};