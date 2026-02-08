#pragma once
#include <cstdint>

namespace SITL_Driver_Configs {
    struct SITL_GPS_Config {
        static constexpr uint8_t NUM_SATELLITES = 12; // Number of satellites in view
    };

    struct SITL_IMU_Config {
        static constexpr uint16_t ACCEL_SCALE = 2048;   // LSB/g
        static constexpr float GYRO_SCALE = 16.4f;      // LSB/(deg/s)

        static constexpr float GRAVITY = 9.81f; // m/s^2
    };

    struct SITL_IWDG_Config {
        static constexpr uint32_t WATCHDOG_TIMEOUT_MS = 10000.0f; // 10 seconds
        static constexpr uint32_t WATCHDOG_CHECK_INTERVAL_MS = 1; // Check every 1 ms
    };

    struct SITL_PowerModule_Config {
        static constexpr float V_FULL = 16.8f;      // 4S fully charged voltage
        static constexpr float V_NOMINAL = 14.8f;   // 4S nominal voltage
        static constexpr float V_EMPTY = 14.0f;     // 4S empty voltage

        static constexpr float MAX_BATTERY_CAPACITY_AH = 5.0f; // 5000mAh
    };
}
