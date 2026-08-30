#pragma once
#include "magnetometer_iface.hpp"
#include "sitl_driver_configs.hpp"
#include <cmath>

class SITL_Magnetometer : public IMagnetometer {
private:
    using Config = SITL_Driver_Configs::SITL_Magnetometer_Config;

    MagData_t magData = {};
    bool sampleReady = false;

public:
    bool init() override {
        magData = {};
        magData.timestamp = 0;
        return true;
    }

    /**
     * Simulates the magnetometer by projecting the WMM reference field into the
     * body frame using the plant's attitude, mirroring how SITL_IMU projects
     * gravity. Body frame is FRD, matching the IMU and the hardware driver.
     */
    void update_from_plant(double roll_rad, double pitch_rad, double yaw_rad) {
        // Reference field in NED
        const float CI = std::cos(Config::INCLINATION_RAD);
        const float SI = std::sin(Config::INCLINATION_RAD);
        const float BN = Config::FIELD_STRENGTH_UT * CI * std::cos(Config::DECLINATION_RAD);
        const float BE = Config::FIELD_STRENGTH_UT * CI * std::sin(Config::DECLINATION_RAD);
        const float BD = Config::FIELD_STRENGTH_UT * SI;

        float sr = std::sin(roll_rad);
        float cr = std::cos(roll_rad);
        float sp = std::sin(pitch_rad);
        float cp = std::cos(pitch_rad);
        float sy = std::sin(yaw_rad);
        float cy = std::cos(yaw_rad);

        // NED -> body (3-2-1 yaw, pitch, roll)
        float bx = cp * cy * BN + cp * sy * BE - sp * BD;
        float by = (sr * sp * cy - cr * sy) * BN + (sr * sp * sy + cr * cy) * BE + sr * cp * BD;
        float bz = (cr * sp * cy + sr * sy) * BN + (cr * sp * sy - sr * cy) * BE + cr * cp * BD;

        magData.x = bx;
        magData.y = by;
        magData.z = bz;

        // Raw LSB counts as the hardware would report them before scaling
        magData.rawX = static_cast<int16_t>(bx / Config::SENS_XY_UT_PER_LSB);
        magData.rawY = static_cast<int16_t>(by / Config::SENS_XY_UT_PER_LSB);
        magData.rawZ = static_cast<int16_t>(bz / Config::SENS_Z_UT_PER_LSB);

        magData.timestamp += 1000 / SITL_Driver_Configs::SITL_DRIVER_UPDATE_RATE_HZ;
        sampleReady = true;
    }

    /**
     * Non blocking, matching mlx90393.cpp: isNew is true only on the read that
     * consumes a fresh sample.
     */
    MagData_t readData() override {
        magData.isNew = sampleReady;
        sampleReady = false;
        return magData;
    }
};
