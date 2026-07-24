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
        bool readData(BaroData_t &data) override {
            data = baroData;
            return true;
        }

        /**
         * Simulates the barometer readings based on the physics engine (Plant)
         * Note: This converts altitude to pressure based on configured temperature
         */
        void update_from_plant(double alt_m) {
            baroData.temperatureC = Config::AMBIENT_TEMP_C;
            baroData.altitude = static_cast<float>(alt_m);

            const float T_K = Config::AMBIENT_TEMP_C + Config::KELVIN_OFFSET;
            const float base = 1.0f - (Config::TEMP_LAPSE_RATE * static_cast<float>(alt_m)) / T_K;

            baroData.pressureKPa = Config::SEA_LEVEL_PRESSURE_KPA * 
                powf(base, 1.0f / Config::BAROMETRIC_EXPONENT);
        }
};
