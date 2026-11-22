#pragma once
#include "airspeed_iface.hpp"

class SITL_Airspeed : public IAirspeed {
    private:
        // Initialized to zero; SITL has no differential-pressure plant model yet.
        AirspeedData_t airspeedData = {};
    public:
        ZP_Error getAirspeedData(AirspeedData_t* data) override {
            if (data == nullptr) return ZP_ERROR_NULLPTR;
            *data = airspeedData;
            return ZP_ERROR_OK;
        }

        // Lets the plant push a simulated airspeed value into the driver.
        void update_from_plant(double airspeed_m_s) {
            airspeedData.airspeed = airspeed_m_s;
        }
};
