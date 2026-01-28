#pragma once
#include <cstdint>
#include <mavlink.h>
#include "config_keys.hpp"

enum class TMSMRequest {
    REQUEST_PARAMS,
};

typedef union TMMessageData_u {
  struct{
      uint8_t baseMode;
      uint32_t customMode;
      uint8_t systemStatus;
  } heartbeatData;
  struct{
      int32_t alt;
      int32_t lat;
      int32_t lon;
      int32_t relativeAlt;
      int16_t vx;
      int16_t vy;
      int16_t vz;
      uint16_t hdg;
  } gposData;
  struct{
      uint16_t roll;
      uint16_t pitch;
      uint16_t yaw;
      uint16_t throttle;
      uint16_t flapAngle;
      uint16_t arm;
  } rcData;
  struct{
      int16_t temperature;
      uint16_t* voltages;
      int16_t currentBattery;
      int32_t currentConsumed;
      int32_t energyConsumed;
      int8_t batteryRemaining;
      int32_t timeRemaining;
      uint8_t chargeState; // 1 = Normal, 2 = Low, 3 = Critical
  } bmData;
  struct{
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
  struct{
      float roll;
      float pitch;
      float yaw;
      float rollspeed;
      float pitchspeed;
      float yawspeed;
  } attitudeData;
  struct{ 
    char key[MAX_KEY_LENGTH];
    float value;
    uint8_t type; // Basically always MAV_PARAM_TYPE_REAL64 which is 64bit float
    uint16_t count;
    uint16_t index;
  } paramData;
} TMMessageData_t;

typedef struct TMMessage{
    enum{
        HEARTBEAT_DATA,
        GPOS_DATA,
        RC_DATA,
        BM_DATA,
        RAW_IMU_DATA,
        ATTITUDE_DATA,
        PARAM_VALUE_DATA
    } dataType;
    TMMessageData_t tmMessageData;
    uint32_t timeBootMs = 0;
} TMMessage_t;

inline TMMessage_t heartbeatPack(uint32_t time_boot_ms, uint8_t base_mode, uint32_t custom_mode, uint8_t system_status) {
    const TMMessageData_t DATA = {.heartbeatData={base_mode, custom_mode, system_status }};
    return TMMessage_t{TMMessage_t::HEARTBEAT_DATA, DATA, time_boot_ms};
}

inline TMMessage_t gposDataPack(uint32_t time_boot_ms, int32_t alt, int32_t lat, int32_t lon, int32_t relative_alt, int16_t vx, int16_t vy, int16_t vz,uint16_t hdg) {
    const TMMessageData_t DATA = {.gposData={alt, lat, lon, relative_alt, vx, vy, vz, hdg }};
    return TMMessage_t{TMMessage_t::GPOS_DATA, DATA, time_boot_ms};
}

inline TMMessage_t rcDataPack(uint32_t time_boot_ms, float roll, float pitch, float yaw, float throttle, float flap_angle, float arm) {
    auto rollPPM = static_cast<uint16_t>(1000 + roll * 10);
    auto pitchPPM = static_cast<uint16_t>(1000 + pitch * 10);
    auto yawPPM = static_cast<uint16_t>(1000 + yaw * 10);
    auto throttlePPM = static_cast<uint16_t>(1000 + throttle * 10);
    auto flapAnglePPM = static_cast<uint16_t>(1000 + flap_angle * 10);
    auto armPPM = static_cast<uint16_t>(1000 + arm * 10);
    const TMMessageData_t DATA = {.rcData ={rollPPM, pitchPPM, yawPPM, throttlePPM, flapAnglePPM, armPPM }};
    return TMMessage_t{TMMessage_t::RC_DATA, DATA, time_boot_ms};
}

inline TMMessage_t bmDataPack(uint32_t time_boot_ms, int16_t temperature, float *voltages, uint8_t voltage_len, int16_t current_battery, int32_t current_consumed,
    int32_t energy_consumed, int8_t battery_remaining, int32_t time_remaining, uint8_t charge_state) {
    uint16_t mavlinkVoltageArray[16] = {UINT16_MAX, UINT16_MAX, UINT16_MAX, UINT16_MAX, UINT16_MAX, UINT16_MAX, UINT16_MAX, UINT16_MAX, UINT16_MAX, UINT16_MAX, UINT16_MAX, UINT16_MAX, UINT16_MAX, UINT16_MAX, UINT16_MAX, UINT16_MAX};
    for (int i = 0; i < voltage_len; i++) {
    	mavlinkVoltageArray[i] = static_cast<uint16_t>(voltages[i]);
    }
    if (temperature == -1) {
        temperature = INT16_MAX;
    }
    const TMMessageData_t DATA = {.bmData ={temperature, mavlinkVoltageArray, current_battery,
    current_consumed, energy_consumed, battery_remaining, time_remaining, charge_state}};
    return TMMessage_t{TMMessage_t::BM_DATA, DATA, time_boot_ms};
}

inline TMMessage_t rawImuDataPack(uint32_t time_boot_ms, int16_t xacc, int16_t yacc, int16_t zacc, int16_t xgyro, int16_t ygyro, int16_t zgyro) {
    int16_t xmag = 0;
    int16_t ymag = 0;
    int16_t zmag = 0;
    uint8_t id = 0;
    int16_t temperature = 0;
    const TMMessageData_t DATA = {.rawImuData ={xacc, yacc, zacc, xgyro, ygyro, zgyro, xmag, ymag, zmag, id, temperature }};
    return TMMessage_t{TMMessage_t::RAW_IMU_DATA, DATA, time_boot_ms};
}

inline TMMessage_t attitudeDataPack(uint32_t time_boot_ms, float roll, float pitch, float yaw) {
    float rollspeed = 0.0f;
    float pitchspeed = 0.0f;
    float yawspeed = 0.0f;
    const TMMessageData_t DATA = {.attitudeData ={roll, pitch, yaw, rollspeed, pitchspeed, yawspeed }};
    return TMMessage_t{TMMessage_t::ATTITUDE_DATA, DATA, time_boot_ms};
}

inline TMMessage_t paramDataPack(uint32_t time_boot_ms, uint16_t index, uint16_t count, Param_t param) {
    TMMessageData_t DATA = {.paramData = { .value = param.value, .type = MAV_PARAM_TYPE_REAL64, .count = count, .index = index}};
    strncpy(DATA.paramData.key, param.key, MAX_KEY_LENGTH - 1);
    DATA.paramData.key[MAX_KEY_LENGTH - 1] = '\0';  // Ensure null termination
    return TMMessage_t{TMMessage_t::PARAM_VALUE_DATA, DATA, time_boot_ms};
}

typedef union TMSMMessage_u {
    struct {
        char keyId[MAX_KEY_LENGTH];
        float value;
    } paramChangeData;
    struct {
        TMSMRequest requestType;
    } requestData;
} TMSMMessageData_t;

typedef struct TMSMMessage{
    enum{
        PARAM_CHANGE_DATA,
        REQUEST_DATA
    } dataType;
    TMSMMessageData_t tmSMMessageData;
    uint32_t timeBootMs = 0;
} TMSMMessage_t;

inline TMSMMessage_t paramChangePack(uint32_t time_boot_ms, const char* keyId, float value) {
    TMSMMessageData_t DATA = {.paramChangeData = {}};
    strncpy(DATA.paramChangeData.keyId, keyId, MAX_KEY_LENGTH - 1);
    DATA.paramChangeData.keyId[MAX_KEY_LENGTH - 1] = '\0';  // Ensure null termination
    DATA.paramChangeData.value = value;
    return TMSMMessage_t{TMSMMessage_t::PARAM_CHANGE_DATA, DATA, time_boot_ms};
}

inline TMSMMessage_t requestPack(uint32_t time_boot_ms, TMSMRequest requestType) {
    const TMSMMessageData_t DATA = {.requestData={.requestType = requestType}};
    return TMSMMessage_t{TMSMMessage_t::REQUEST_DATA, DATA, time_boot_ms};
}
