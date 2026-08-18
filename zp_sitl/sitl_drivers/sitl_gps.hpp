#pragma once
#include "gps_iface.hpp"
#include "sitl_driver_configs.hpp"

class SITL_GPS : public IGPS {
private:
    using Config = SITL_Driver_Configs::SITL_GPS_Config;

    GpsData_t gpsData = {{0,0,0,0,0,0}, 0.0f, 0.0f, 0.0f, 0, 0.0f, 0.0f, false, 0.0f, 0.0f, 0.0f};
    uint32_t usSinceFrame = 0;

public:
    void update_from_plant(double lat_deg, double lon_deg, double alt_m, double ground_speed_mps, double course_deg,
                           double vx_mps, double vy_mps, double vz_mps, uint32_t dt_us) {
        usSinceFrame += dt_us;
        if (usSinceFrame < Config::FRAME_PERIOD_US) {
            return;
        }
        usSinceFrame = 0;

        gpsData.latitude = lat_deg;
        gpsData.longitude = lon_deg;
        gpsData.altitude = alt_m;
        gpsData.groundSpeed = ground_speed_mps * 100.0f;  // m/s to cm/s
        gpsData.trackAngle = course_deg;
        gpsData.vx = vx_mps;
        gpsData.vy = vy_mps;
        gpsData.vz = vz_mps;
        gpsData.isNew = true;
        gpsData.numSatellites = Config::NUM_SATELLITES;
    }
    
    GpsData_t readData() override {
        GpsData_t sample = gpsData;
        gpsData.isNew = false;
        return sample;
    }
};
