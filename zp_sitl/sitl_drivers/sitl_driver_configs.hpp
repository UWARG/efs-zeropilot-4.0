#pragma once
#include <cstdint>

namespace SITL_Driver_Configs {
    static constexpr uint32_t SITL_DRIVER_UPDATE_RATE_HZ = 1000; // 1 kHz update rate for all SITL drivers

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
        static constexpr float V_EMPTY = 12.0f;     // 4S empty voltage

        static constexpr float MAX_BATTERY_CAPACITY_MAH = 5000.0f; // Battery capacity in mAh

        static constexpr float BATTERY_INTERNAL_RESISTANCE_OHMS = 0.020f; // Simulated internal resistance of the battery in Ohms

        static constexpr float CURRENT_DRAW_IDLE = 3.0f; // Simulated current draw at idle in Amps
        static constexpr float CURRENT_DRAW_PER_RPM = 0.012f; // Simulated current draw per RPM in Amps

        static constexpr float AMBIENT_TEMP_C = 25.0f; // Base ambient room temperature in °C
        static constexpr float TEMP_RISE_PER_AMP = 0.3f; // Steady-state heating factor in °C per Ampere
        static constexpr float TEMP_THERMAL_LAG_COEFF = 0.05f; // First-order filter coefficient (0.0 to 1.0) for thermal mass inertia
    };

    struct SITL_Barometer_Config {
        static constexpr float AMBIENT_TEMP_C = 25.0f; // Base ambient room temperature in °C
        static constexpr float KELVIN_OFFSET = 273.15f;
        static constexpr float TEMP_LAPSE_RATE = 0.0065f; // K/m
        static constexpr float SEA_LEVEL_PRESSURE_KPA = 101.325f;
        static constexpr float BAROMETRIC_EXPONENT = 0.190284f; // must match the driver's exponent
    };

    struct SITL_TELEM_Config {
        static constexpr uint32_t RX_BUF_SZ_BYTES = 1048576; // 1 MB receive buffer
    };
}
