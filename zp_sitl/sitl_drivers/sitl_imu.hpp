#pragma once
#include "imu_iface.hpp"
#include "sitl_driver_configs.hpp"
#include "unit_conversions.hpp"
#include <cmath>

class SITL_IMU : public IIMU {
private:
    using Config = SITL_Driver_Configs::SITL_IMU_Config;

    RawImu_t rawData = {};
    ScaledImu_t scaledData = {};
    RawImuBatch_t rawBatch = {&rawData, 1}; // Only returning 1 data packet for sitl
    ScaledImuBatch_t scaledBatch = {&scaledData, 1}; // Only returning 1 data packet for sitl

    // Constants for internal conversions
    static constexpr float RAD_TO_DEG = 57.2957795f;
    static constexpr float DEG_TO_RAD = 0.0174532925f;


    // Convert a float value to a 16-bit signed integer, clamping the value if it's out of range
    static int16_t toRawCount(float value) {
        if (value > 32767.0f) return 32767;
        if (value < -32768.0f) return -32768;
        return (int16_t)value;
    }

public:
    int init() override {
        rawData.timestamp = 0; // Initialize timestamp
        return 0; // Success
    }
    
    /**
     * Simulates the IMU readings based on the physics engine (Plant)
     * Note: This converts physical SI units into "Raw" LSB counts
    */
    void update_from_plant(double roll_rad, double pitch_rad, double p_rad_s, double q_rad_s, double r_rad_s,
                           double ax_body, double ay_body, double az_body, uint32_t dt_us) {
        // Pre-calculate trig to save cycles
        float sr = std::sin(roll_rad);
        float cr = std::cos(roll_rad);
        float sp = std::sin(pitch_rad);
        float cp = std::cos(pitch_rad);

        // Accelerometer: Gravity projection
        // LSB_PER_G = ACCEL_SCALE (e.g., 2048)
        float ax = (float)ax_body + Config::GRAVITY * sp;
        float ay = (float)ay_body - Config::GRAVITY * sr * cp;
        float az = (float)az_body - Config::GRAVITY * cr * cp;

        // Convert m/s^2 to LSB: (Value / 9.81) * Scale_Factor
        constexpr float ACCEL_TO_LSB = (float)Config::ACCEL_SCALE / Config::GRAVITY;
        rawData.xacc = toRawCount(ax * ACCEL_TO_LSB);
        rawData.yacc = toRawCount(ay * ACCEL_TO_LSB);
        rawData.zacc = toRawCount(az * ACCEL_TO_LSB);

        // Gyro: Convert rad/s to deg/s then to LSB
        float p_deg_s = (float)p_rad_s * RAD_TO_DEG;
        float q_deg_s = (float)q_rad_s * RAD_TO_DEG;
        float r_deg_s = (float)r_rad_s * RAD_TO_DEG;

        rawData.xgyro = toRawCount(p_deg_s * Config::GYRO_SCALE);
        rawData.ygyro = toRawCount(q_deg_s * Config::GYRO_SCALE);
        rawData.zgyro = toRawCount(r_deg_s * Config::GYRO_SCALE);

        // Advance by however long the plant actually stepped, so AM's dt matches the sim instead of a fixed 1 ms
        rawData.timestamp += dt_us;
    }
    
    RawImuBatch_t readRawData() override {
        return rawBatch;
    }

    // Nominal rate: AM asks for this once at construction, before any plant step has been measured
    float getODRHz() override {
        return (float)SITL_Driver_Configs::SITL_DRIVER_UPDATE_RATE_HZ;
    }

    GyroBias_t getGyroStartupBias(uint8_t imuId) override {
        return GyroBias_t{0.0f, 0.0f, 0.0f}; // No startup bias in simulation
    }

    /**
     * Reverses the raw data back into meaningful SI units (m/s^2 and rad/s)
     */
    ScaledImuBatch_t scaleIMUData(const RawImuBatch_t &rawDataBatch) override {
        for (int i = 0; i < rawDataBatch.count; i++) {
            const RawImu_t &raw = rawDataBatch.data[i];

            // Convert LSB back to m/s^2: (Raw / Scale) * 9.81
            scaledData.xacc = ((float)raw.xacc / Config::ACCEL_SCALE) * Config::GRAVITY;
            scaledData.yacc = ((float)raw.yacc / Config::ACCEL_SCALE) * Config::GRAVITY;
            scaledData.zacc = ((float)raw.zacc / Config::ACCEL_SCALE) * Config::GRAVITY;

            // Convert LSB back to rad/s (consistent with hardware IMU driver)
            scaledData.xgyro = (float)raw.xgyro / Config::GYRO_SCALE * ZP_UNITS::DEG_TO_RAD;
            scaledData.ygyro = (float)raw.ygyro / Config::GYRO_SCALE * ZP_UNITS::DEG_TO_RAD;
            scaledData.zgyro = (float)raw.zgyro / Config::GYRO_SCALE * ZP_UNITS::DEG_TO_RAD;

            scaledData.timestamp = raw.timestamp;
        }
        return scaledBatch; 
    }
};
