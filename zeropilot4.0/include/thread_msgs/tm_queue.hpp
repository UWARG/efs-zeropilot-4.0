#pragma once
#include <cstdint>
#include <string.h>
#include "zp_error.h"

static constexpr uint8_t TM_QUEUE_STATUSTEXT_CHAR_COUNT = 50;
static constexpr uint8_t TM_QUEUE_RC_CHANNELS_COUNT = 18;
static constexpr uint8_t TM_QUEUE_BATTERY_VOLTAGES_COUNT = 10;

typedef union TMMessageData_u {
  struct {
      uint8_t baseMode;
      uint32_t customMode;
      uint8_t systemStatus;
  } heartbeatData;
  struct {
      uint8_t severity;
      char text[TM_QUEUE_STATUSTEXT_CHAR_COUNT];
      uint16_t id;
      uint8_t chunkSeq;
  } statusTextData;
  struct {
      uint8_t fixType;
      int32_t lat;
      int32_t lon;
      int32_t alt;
      uint16_t eph;
      uint16_t epv;
      uint16_t vel;
      uint16_t cog;
      uint8_t satellitesVisible;
      int32_t altEllipsoid;
      uint32_t hAcc;
      uint32_t vAcc;
      uint32_t velAcc;
      uint32_t hdgAcc;
      uint16_t yaw;
  } gpsRawData;

  struct {
      uint8_t port;
      uint16_t servo1Raw;
      uint16_t servo2Raw;
      uint16_t servo3Raw;
      uint16_t servo4Raw;
      uint16_t servo5Raw;
      uint16_t servo6Raw;
      uint16_t servo7Raw;
      uint16_t servo8Raw;
      uint16_t servo9Raw;
      uint16_t servo10Raw;
      uint16_t servo11Raw;
      uint16_t servo12Raw;
      uint16_t servo13Raw;
      uint16_t servo14Raw;
      uint16_t servo15Raw;
      uint16_t servo16Raw;
  } servoOutputRawData;
  struct {
      uint8_t channelCount;
      uint16_t channels[TM_QUEUE_RC_CHANNELS_COUNT];
  } rcData;
  struct {
      uint8_t batteryId;
      int16_t temperature;
      uint16_t voltages[TM_QUEUE_BATTERY_VOLTAGES_COUNT];
      int16_t currentBattery;
      int32_t currentConsumed;
      int32_t energyConsumed;
      int8_t batteryRemaining;
      int32_t timeRemaining;
      uint8_t chargeState; // MAV_BATTERY_CHARGE_STATE
  } batteryData;
  struct {
      int16_t xacc;
      int16_t yacc;
      int16_t zacc;
      int16_t xgyro;
      int16_t ygyro;
      int16_t zgyro;
      int16_t xmag;
      int16_t ymag;
      int16_t zmag;
      uint8_t id;
      int16_t temperature;
  } rawImuData;
  struct {
      float roll;
      float pitch;
      float yaw;
      float rollspeed;
      float pitchspeed;
      float yawspeed;
  } attitudeData;

  struct {
    float pressAbs;
    float pressDiff;
    int16_t temperature;
    int16_t temperaturePressDiff;
  } scaledPressureData;

  struct {
    uint16_t minDistance;
    uint16_t maxDistance;
    uint16_t currentDistance;
    uint8_t id;
    uint8_t covariance;
    float horizontalFov;
    float verticalFov;
    float quaternion[4];
    uint8_t signalQuality;
  } distanceSensorData;

  struct {
    uint32_t sensorsPresent;
    uint32_t sensorsEnabled;
    uint32_t sensorsHealth;
    uint16_t load;
    uint16_t voltageBattery;
    int16_t currentBattery;
    int8_t batteryRemaining;
  } sysStatusData;
} TMMessageData_t;

typedef struct TMMessage{
    enum{
        HEARTBEAT_DATA,
        STATUSTEXT_DATA,
        GPS_RAW_DATA,
        SERVO_OUTPUT_RAW,
        RC_DATA,
        BATTERY_DATA,
        RAW_IMU_DATA,
        ATTITUDE_DATA,
        SCALED_PRESSURE_DATA,
        DISTANCE_SENSOR_DATA,
        SYS_STATUS_DATA
    } dataType;
    TMMessageData_t tmMessageData;
    uint32_t timeBootMs = 0;
} TMMessage_t;

inline ZP_Error heartbeatPack(TMMessage_t &data, uint32_t time_boot_ms, uint8_t base_mode, uint32_t custom_mode, uint8_t system_status) {
    const TMMessageData_t DATA = {.heartbeatData={base_mode, custom_mode, system_status}};
    data = TMMessage_t{TMMessage_t::HEARTBEAT_DATA, DATA, time_boot_ms};
    return ZP_ERROR_OK;
}

inline ZP_Error statusTextPack(TMMessage_t &data, uint32_t time_boot_ms, uint8_t severity, const char text[TM_QUEUE_STATUSTEXT_CHAR_COUNT], uint16_t id, uint8_t chunk_seq) {
    ZP_Error result = ZP_ERROR_OK;
    if (text == nullptr) {
        result |= ZP_ERROR_NULLPTR;
    }
    else {
        TMMessageData_t msgData = {.statusTextData={severity, "", id, chunk_seq}};
        constexpr size_t MAX_LEN = sizeof(msgData.statusTextData.text) - 1; // Reserve space for null terminator

        // Get length in a firmware safe way without using strlen which may read out of bounds if text is not null terminated
        size_t len = 0;
        while (len < MAX_LEN && text[len] != '\0') ++len;

        memcpy(msgData.statusTextData.text, text, len); // Copy text without null terminator
        msgData.statusTextData.text[len] = '\0'; // Ensure null termination

        data = TMMessage_t{TMMessage_t::STATUSTEXT_DATA, msgData, time_boot_ms};
    }
    return result;
}

inline ZP_Error gpsRawDataPack(TMMessage_t &data, uint32_t time_boot_ms, uint8_t fix_type, int32_t lat, int32_t lon, int32_t alt,
                                 uint16_t eph, uint16_t epv, uint16_t vel, uint16_t cog, uint8_t satellites_visible,
                                 int32_t alt_el = 0, uint32_t h_acc = 0, uint32_t v_acc = 0,
                                 uint32_t vel_acc = 0, uint32_t hdg_acc = 0, uint16_t yaw = 0) {
    const TMMessageData_t DATA = {
        .gpsRawData = {
            fix_type, lat, lon, alt, eph, epv, vel, cog, satellites_visible,
            alt_el, h_acc, v_acc, vel_acc, hdg_acc, yaw
        }
    };
    data = TMMessage_t{TMMessage_t::GPS_RAW_DATA, DATA, time_boot_ms};
    return ZP_ERROR_OK;
}

inline ZP_Error scaledPressurePack(TMMessage_t &data, uint32_t time_boot_ms, float press_abs_kpa, float press_diff_kpa,
                                     float temperature_degC, float temperature_press_diff_degC) {
    float pressAbs = press_abs_kpa * 10.0f; // kPa -> hPa
    float pressDiff = press_diff_kpa * 10.0f; // kPa -> hPa
    int16_t temp = static_cast<int16_t>(temperature_degC * 100.0f); // C -> cC
    int16_t tempPressDiff = static_cast<int16_t>(temperature_press_diff_degC * 100.0f); // C -> cC

    const TMMessageData_t DATA = {
        .scaledPressureData = {
            pressAbs, pressDiff, temp, tempPressDiff
        }
    };
    data = TMMessage_t{TMMessage_t::SCALED_PRESSURE_DATA, DATA, time_boot_ms};
    return ZP_ERROR_OK;
}

inline ZP_Error servoOutputRawPack(TMMessage_t &data, uint32_t time_boot_ms, uint8_t port, const uint16_t servo_values[16]) {
    ZP_Error result = ZP_ERROR_OK;
    if (servo_values == nullptr) {
        result |= ZP_ERROR_NULLPTR;
    } else {
        const TMMessageData_t DATA = {
            .servoOutputRawData = {
                port,
                servo_values[0], servo_values[1], servo_values[2], servo_values[3],
                servo_values[4], servo_values[5], servo_values[6], servo_values[7],
                servo_values[8], servo_values[9], servo_values[10], servo_values[11],
                servo_values[12], servo_values[13], servo_values[14], servo_values[15]
            }
        };
        data = TMMessage_t{TMMessage_t::SERVO_OUTPUT_RAW, DATA, time_boot_ms};
    }
    return result;
}

inline ZP_Error rcDataPack(TMMessage_t &data, uint32_t time_boot_ms, const float* controlSignals, uint8_t size) {
    ZP_Error result = ZP_ERROR_OK;
    if (controlSignals == nullptr) {
        result |= ZP_ERROR_NULLPTR;
    }
    if (size > TM_QUEUE_RC_CHANNELS_COUNT) {
        result |= ZP_ERROR_RANGE;
    }

    if (result == ZP_ERROR_OK) {
        TMMessageData_t msgData;
        msgData.rcData.channelCount = size;
        for (int i = 0; i < TM_QUEUE_RC_CHANNELS_COUNT; i++) {
            msgData.rcData.channels[i] = (i < size) ? static_cast<uint16_t>(1000 + controlSignals[i] * 10) : UINT16_MAX;
        }
        data = TMMessage_t{TMMessage_t::RC_DATA, msgData, time_boot_ms};
    }
    return result;
}

inline ZP_Error batteryDataPack(TMMessage_t &data, uint32_t time_boot_ms, uint8_t battery_id, float temperature,
                                  float *voltages, uint8_t voltage_len, float current_instantaneous,
                                  float charge_accumulated, float energy_consumed, int8_t battery_remaining,
                                  int32_t time_remaining, uint8_t charge_state) {
    ZP_Error result = ZP_ERROR_OK;
    if (voltages == nullptr) {
        result |= ZP_ERROR_NULLPTR;
    }
    if (voltage_len > TM_QUEUE_BATTERY_VOLTAGES_COUNT) {
        result |= ZP_ERROR_RANGE;
    }

    if (result == ZP_ERROR_OK) {
        int16_t scaledTemperature = static_cast<int16_t>(temperature * 100); // C -> cC
        int16_t scaledCurrentBattery = static_cast<int16_t>(current_instantaneous * 100); // A -> cA
        int32_t scaledCurrentConsumed = static_cast<int32_t>((charge_accumulated * 1000) / 3600); // C -> mAh
        int32_t scaledEnergyConsumed = static_cast<int32_t>(energy_consumed / 100); // J -> hJ

        TMMessage_t msg;
        msg.dataType = TMMessage_t::BATTERY_DATA;
        msg.timeBootMs = time_boot_ms;

        auto& battData = msg.tmMessageData.batteryData;
        battData.batteryId = battery_id;
        battData.temperature = scaledTemperature;
        battData.currentBattery = scaledCurrentBattery;
        battData.currentConsumed = scaledCurrentConsumed;
        battData.energyConsumed = scaledEnergyConsumed;
        battData.batteryRemaining = battery_remaining;
        battData.timeRemaining = time_remaining;
        battData.chargeState = charge_state;

        for (int i = 0; i < TM_QUEUE_BATTERY_VOLTAGES_COUNT; i++) {
            battData.voltages[i] = UINT16_MAX;
        }

        for (int i = 0; i < voltage_len && i < TM_QUEUE_BATTERY_VOLTAGES_COUNT; i++) {
            battData.voltages[i] = static_cast<uint16_t>(voltages[i] * 1000.0); // V -> mV
        }

        data = msg;
    }
    return result;
}

inline ZP_Error rawImuDataPack(TMMessage_t &data, uint32_t time_boot_ms, int16_t xacc, int16_t yacc, int16_t zacc, int16_t xgyro, int16_t ygyro, int16_t zgyro) {
    int16_t xmag = 0;
    int16_t ymag = 0;
    int16_t zmag = 0;
    uint8_t id = 0;
    int16_t temperature = 0;
    const TMMessageData_t DATA = {
        .rawImuData = {
            xacc, yacc, zacc, xgyro, ygyro, zgyro, xmag, ymag, zmag, id, temperature
        }
    };
    data = TMMessage_t{TMMessage_t::RAW_IMU_DATA, DATA, time_boot_ms};
    return ZP_ERROR_OK;
}

inline ZP_Error attitudeDataPack(TMMessage_t &data, uint32_t time_boot_ms, float roll, float pitch, float yaw) {
    float rollspeed = 0.0f;
    float pitchspeed = 0.0f;
    float yawspeed = 0.0f;
    const TMMessageData_t DATA = {
        .attitudeData = {
            roll, pitch, yaw, rollspeed, pitchspeed, yawspeed
        }
    };
    data = TMMessage_t{TMMessage_t::ATTITUDE_DATA, DATA, time_boot_ms};
    return ZP_ERROR_OK;
}

inline ZP_Error distanceSensorDataPack(TMMessage_t &data, uint32_t time_boot_ms, float min_distance, float max_distance, float current_distance, uint8_t id, float covariance, float horizontal_fov_rad, float vertical_fov_rad, float quaternion[4], uint8_t signal_quality_pct) {
    ZP_Error result = ZP_ERROR_OK;
    if (quaternion == nullptr) {
        result |= ZP_ERROR_NULLPTR;
    } else {
        uint16_t scaledMinDistance = static_cast<uint16_t>(min_distance * 100); // m -> cm
        uint16_t scaledMaxDistance = static_cast<uint16_t>(max_distance * 100); // m -> cm
        uint16_t scaledCurrentDistance = static_cast<uint16_t>(current_distance * 100); // m -> cm
        uint8_t scaledCovariance = static_cast<uint8_t>(covariance * 10000); // m^2 -> cm^2

        const TMMessageData_t DATA = {
            .distanceSensorData = {
                scaledMinDistance, scaledMaxDistance, scaledCurrentDistance, id, scaledCovariance, horizontal_fov_rad, vertical_fov_rad,
                {quaternion[0], quaternion[1], quaternion[2], quaternion[3]}, signal_quality_pct
            }
        };
        data = TMMessage_t{TMMessage_t::DISTANCE_SENSOR_DATA, DATA, time_boot_ms};
    }
    return result;
}

inline ZP_Error sysStatusPack(TMMessage_t &data, uint32_t time_boot_ms, uint32_t sensors_present,
                                uint32_t sensors_enabled, uint32_t sensors_health, uint16_t load,
                                float voltage_battery, float current_battery, int8_t battery_remaining) {
    const uint16_t SCALED_VOLTAGE = static_cast<uint16_t>(voltage_battery * 1000.0f); // V -> mV
    const int16_t SCALED_CURRENT = static_cast<int16_t>(current_battery * 100.0f);    // A -> cA

    const TMMessageData_t DATA = {
        .sysStatusData = {
            sensors_present, sensors_enabled, sensors_health,
            load, SCALED_VOLTAGE, SCALED_CURRENT, battery_remaining
        }
    };
    data = TMMessage_t{TMMessage_t::SYS_STATUS_DATA, DATA, time_boot_ms};
    return ZP_ERROR_OK;
}
