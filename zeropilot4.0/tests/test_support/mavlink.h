#pragma once

#include <cstdint>
#include <cstring>

typedef uint8_t MAV_STATE;
typedef uint8_t MAV_SEVERITY;
typedef uint8_t MAV_BATTERY_CHARGE_STATE;

static constexpr uint8_t MAV_MODE_FLAG_CUSTOM_MODE_ENABLED = 1 << 0;
static constexpr uint8_t MAV_MODE_FLAG_SAFETY_ARMED = 1 << 7;

static constexpr MAV_STATE MAV_STATE_STANDBY = 3;
static constexpr MAV_STATE MAV_STATE_ACTIVE = 4;
static constexpr MAV_STATE MAV_STATE_CRITICAL = 5;

static constexpr MAV_SEVERITY MAV_SEVERITY_CRITICAL = 2;
static constexpr MAV_SEVERITY MAV_SEVERITY_WARNING = 4;
static constexpr MAV_SEVERITY MAV_SEVERITY_INFO = 6;

static constexpr MAV_BATTERY_CHARGE_STATE MAV_BATTERY_CHARGE_STATE_OK = 1;
static constexpr MAV_BATTERY_CHARGE_STATE MAV_BATTERY_CHARGE_STATE_LOW = 2;
static constexpr MAV_BATTERY_CHARGE_STATE MAV_BATTERY_CHARGE_STATE_CRITICAL = 3;

static constexpr uint8_t MAV_TYPE_FIXED_WING = 1;
static constexpr uint8_t MAV_AUTOPILOT_ARDUPILOTMEGA = 3;
static constexpr uint32_t MAV_BATTERY_FAULT_DEEP_DISCHARGE = 1u << 0;
static constexpr uint8_t MAV_BATTERY_FUNCTION_ALL = 0;
static constexpr uint8_t MAV_BATTERY_TYPE_LIPO = 0;

static constexpr uint32_t MAVLINK_MSG_ID_HEARTBEAT = 0;
static constexpr uint32_t MAVLINK_MSG_ID_PARAM_SET = 23;
static constexpr uint32_t MAVLINK_MSG_ID_STATUSTEXT = 253;
static constexpr uint32_t MAVLINK_MSG_ID_GPS_RAW_INT = 24;
static constexpr uint32_t MAVLINK_MSG_ID_SERVO_OUTPUT_RAW = 36;
static constexpr uint32_t MAVLINK_MSG_ID_RC_CHANNELS = 65;
static constexpr uint32_t MAVLINK_MSG_ID_BATTERY_STATUS = 147;
static constexpr uint32_t MAVLINK_MSG_ID_RAW_IMU = 27;
static constexpr uint32_t MAVLINK_MSG_ID_ATTITUDE = 30;
static constexpr uint32_t MAVLINK_MSG_ID_PARAM_VALUE = 22;

typedef struct mavlink_message {
    uint16_t len = 0;
    uint32_t msgid = 0;
    uint8_t payload[280] = {};
} mavlink_message_t;

typedef struct mavlink_status {
    uint8_t flags = 0;
} mavlink_status_t;

template <typename... Args>
inline uint16_t mavlink_msg_heartbeat_pack(uint8_t, uint8_t, mavlink_message_t *msg, Args...) {
    msg->msgid = MAVLINK_MSG_ID_HEARTBEAT;
    msg->len = 9;
    return msg->len;
}

template <typename... Args>
inline uint16_t mavlink_msg_statustext_pack(uint8_t, uint8_t, mavlink_message_t *msg, Args...) {
    msg->msgid = MAVLINK_MSG_ID_STATUSTEXT;
    msg->len = 10;
    return msg->len;
}

template <typename... Args>
inline uint16_t mavlink_msg_gps_raw_int_pack(uint8_t, uint8_t, mavlink_message_t *msg, Args...) {
    msg->msgid = MAVLINK_MSG_ID_GPS_RAW_INT;
    msg->len = 12;
    return msg->len;
}

template <typename... Args>
inline uint16_t mavlink_msg_servo_output_raw_pack(uint8_t, uint8_t, mavlink_message_t *msg, Args...) {
    msg->msgid = MAVLINK_MSG_ID_SERVO_OUTPUT_RAW;
    msg->len = 14;
    return msg->len;
}

inline uint16_t mavlink_msg_battery_status_pack(
    uint8_t, uint8_t, mavlink_message_t *msg,
    uint8_t, uint8_t, uint8_t, int16_t, const uint16_t *,
    int16_t, int32_t, int32_t, int8_t, int32_t, uint8_t,
    const uint16_t *, uint8_t, uint32_t) {
    msg->msgid = MAVLINK_MSG_ID_BATTERY_STATUS;
    msg->len = 16;
    return msg->len;
}

template <typename... Args>
inline uint16_t mavlink_msg_raw_imu_pack(uint8_t, uint8_t, mavlink_message_t *msg, Args...) {
    msg->msgid = MAVLINK_MSG_ID_RAW_IMU;
    msg->len = 18;
    return msg->len;
}

template <typename... Args>
inline uint16_t mavlink_msg_attitude_pack(uint8_t, uint8_t, mavlink_message_t *msg, Args...) {
    msg->msgid = MAVLINK_MSG_ID_ATTITUDE;
    msg->len = 20;
    return msg->len;
}

template <typename... Args>
inline uint16_t mavlink_msg_rc_channels_pack(uint8_t, uint8_t, mavlink_message_t *msg, Args...) {
    msg->msgid = MAVLINK_MSG_ID_RC_CHANNELS;
    msg->len = 22;
    return msg->len;
}

template <typename... Args>
inline uint16_t mavlink_msg_param_value_pack(uint8_t, uint8_t, mavlink_message_t *msg, Args...) {
    msg->msgid = MAVLINK_MSG_ID_PARAM_VALUE;
    msg->len = 8;
    return msg->len;
}

inline uint16_t mavlink_msg_to_send_buffer(uint8_t *buffer, const mavlink_message_t *msg) {
    const uint16_t len = (msg->len == 0) ? 1 : msg->len;
    std::memset(buffer, 0, len);
    return len;
}

inline bool mavlink_parse_char(uint8_t, uint8_t, mavlink_message_t *, mavlink_status_t *) {
    return false;
}

inline float mavlink_msg_param_set_get_param_value(const mavlink_message_t *) {
    return 0.0f;
}

inline uint8_t mavlink_msg_param_set_get_param_type(const mavlink_message_t *) {
    return 0;
}
