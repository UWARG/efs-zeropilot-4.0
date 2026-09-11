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
    static constexpr uint16_t SITL_BATCH_PACKETS = 1;
    RawImuBatch_t rawBatch = {&rawData, SITL_BATCH_PACKETS};
    ScaledImuBatch_t scaledBatch = {&scaledData, SITL_BATCH_PACKETS};

    // Constants for internal conversions
    static constexpr float RAD_TO_DEG = 57.2957795f;
    static constexpr float DEG_TO_RAD = 0.0174532925f;

public:
    ZP_Error init() override {
        rawData.timestamp = 0; // Initialize timestamp
        return ZP_ERROR_OK;
    }
    
    /**
     * Simulates the IMU readings based on the physics engine (Plant)
     * Note: This converts physical SI units into "Raw" LSB counts
     */
    void update_from_plant(double roll_rad, double pitch_rad, double p_rad_s, double q_rad_s, double r_rad_s) {
        // Pre-calculate trig to save cycles
        float sr = std::sin(roll_rad);
        float cr = std::cos(roll_rad);
        float sp = std::sin(pitch_rad);
        float cp = std::cos(pitch_rad);

        // Accelerometer: Gravity projection (assuming 1g static)
        // LSB_PER_G = ACCEL_SCALE (e.g., 2048)
        float ax = Config::GRAVITY * sp;
        float ay = -Config::GRAVITY * sr * cp;
        float az = -Config::GRAVITY * cr * cp;
        
        // Convert m/s^2 to LSB: (Value / 9.81) * Scale_Factor
        constexpr float ACCEL_TO_LSB = (float)Config::ACCEL_SCALE / Config::GRAVITY;
        rawData.xacc = (int16_t)(ax * ACCEL_TO_LSB);
        rawData.yacc = (int16_t)(ay * ACCEL_TO_LSB);
        rawData.zacc = (int16_t)(az * ACCEL_TO_LSB);

        // Gyro: Convert rad/s to deg/s then to LSB
        float p_deg_s = (float)p_rad_s * RAD_TO_DEG;
        float q_deg_s = (float)q_rad_s * RAD_TO_DEG;
        float r_deg_s = (float)r_rad_s * RAD_TO_DEG;

        rawData.xgyro = (int16_t)(p_deg_s * Config::GYRO_SCALE);
        rawData.ygyro = (int16_t)(q_deg_s * Config::GYRO_SCALE);
        rawData.zgyro = (int16_t)(r_deg_s * Config::GYRO_SCALE);

        rawData.timestamp += SITL_Driver_Configs::SITL_DRIVER_UPDATE_RATE_HZ; // Increment timestamp for simulation
    }
    
    ZP_Error readRawData(RawImuBatch_t &rawDataBatch) override {
        rawDataBatch = rawBatch;
        return ZP_ERROR_OK;
    }

    float getODRHz() override {
        return (float)SITL_Driver_Configs::SITL_DRIVER_UPDATE_RATE_HZ;
    }

    GyroBias_t getGyroStartupBias(uint8_t imuId) override {
        return GyroBias_t{0.0f, 0.0f, 0.0f}; // No startup bias in simulation
    }

    /**
     * Reverses the raw data back into meaningful SI units (m/s^2 and rad/s)
     */
    ZP_Error scaleIMUData(const RawImuBatch_t &rawDataBatch, ScaledImuBatch_t &scaledDataBatch) override {
        if (rawDataBatch.data == nullptr) {
            return ZP_ERROR_NULLPTR;
        }
        if (rawDataBatch.count > SITL_BATCH_PACKETS) {
            return ZP_ERROR_RANGE;
        }

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

        scaledDataBatch = scaledBatch;
        return ZP_ERROR_OK;
    }
};
