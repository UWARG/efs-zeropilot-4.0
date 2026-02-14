#pragma once
#include "gps_iface.hpp"
#include "sitl_driver_configs.hpp"

class SITL_GPS : public IGPS {
private:
    using Config = SITL_Driver_Configs::SITL_GPS_Config;

    GpsData_t gpsData = {{0,0,0,0,0,0}, 0.0f, 0.0f, 0.0f, 0, 0.0f, 0.0f, false, 0.0f, 0.0f, 0.0f};
    
public:
    void update_from_plant(double lat_deg, double lon_deg, double alt_m, double ground_speed_mps, double course_deg) {
        gpsData.latitude = lat_deg;
        gpsData.longitude = lon_deg;
        gpsData.altitude = alt_m;
        gpsData.groundSpeed = ground_speed_mps * 100.0f;  // m/s to cm/s
        gpsData.trackAngle = course_deg;
        gpsData.vx = 0;
        gpsData.vy = 0;
        gpsData.vz = 0;
        gpsData.isNew = true;
        gpsData.numSatellites = Config::NUM_SATELLITES;
    }
    
    GpsData_t readData() override {
        return gpsData;
    }
};
