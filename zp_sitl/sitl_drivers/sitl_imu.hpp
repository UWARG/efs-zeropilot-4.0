#pragma once
#include "imu_iface.hpp"
#include <cmath>

class SITL_IMU : public IIMU {
private:
    RawImu_t rawData = {0, 0, 0, 0, 0, 0};
    
public:
    int init() override { return 0; }
    
    void update_from_plant(double roll_rad, double pitch_rad, double p_rad_s, double q_rad_s, double r_rad_s) {
        // Accelerometer: gravity projection (2048 LSB/g)
        constexpr float ACCEL_SCALE = 2048.0f / 9.81f;
        float ax = -9.81f * std::sin(pitch_rad);
        float ay = 9.81f * std::sin(roll_rad) * std::cos(pitch_rad);
        float az = 9.81f * std::cos(roll_rad) * std::cos(pitch_rad);
        
        rawData.xacc = (int16_t)(ax * ACCEL_SCALE);
        rawData.yacc = (int16_t)(ay * ACCEL_SCALE);
        rawData.zacc = (int16_t)(az * ACCEL_SCALE);
        
        // Gyro: 16.4 LSB/(deg/s)
        constexpr float GYRO_SCALE = 16.4f;
        constexpr float RAD_TO_DEG = 57.2958f;
        rawData.xgyro = (int16_t)(p_rad_s * RAD_TO_DEG * GYRO_SCALE);
        rawData.ygyro = (int16_t)(q_rad_s * RAD_TO_DEG * GYRO_SCALE);
        rawData.zgyro = (int16_t)(r_rad_s * RAD_TO_DEG * GYRO_SCALE);
    }
    
    RawImu_t readRawData() override {
        return rawData;
    }
    
    ScaledImu_t scaleIMUData(const RawImu_t &raw) override {
        ScaledImu_t scaled;
        scaled.xacc = raw.xacc;
        scaled.yacc = raw.yacc;
        scaled.zacc = raw.zacc;
        scaled.xgyro = raw.xgyro;
        scaled.ygyro = raw.ygyro;
        scaled.zgyro = raw.zgyro;
        return scaled;
    }
};
