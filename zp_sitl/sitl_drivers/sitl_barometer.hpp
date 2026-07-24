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
         * Simulates the barometer readings from the plant's real local atmosphere.
         * Mirrors icp_20100.cpp's pressure/temperature -> altitude formula so SITL
         * matches real hardware
         */
        void update_from_plant(double pressure_kPa, double temperature_C) {
            baroData.pressureKPa = static_cast<float>(pressure_kPa);
            baroData.temperatureC = static_cast<float>(temperature_C);

            baroData.altitude = ((baroData.temperatureC + Config::KELVIN_OFFSET) / Config::TEMP_LAPSE_RATE) *
                (1.0f - powf(baroData.pressureKPa / Config::SEA_LEVEL_PRESSURE_KPA, Config::BAROMETRIC_EXPONENT));
        }
};
