#pragma once
#include <cstdint>
#include <string.h> 

typedef union SMMessageData {
    struct {
        float instance,
        float throttleType,
        float throttle,
        float timeout,
        int32_t motorCount,
        int32_t testOrder
    } motorTestCmd;
    struct {
        int32_t accelCalibration,
        int32_t compassCalibration
    } calibrationCmd;
    struct {
        float switchState
    } setSafetySwitchCmd;
    struct {
        float autopilotAction,
        float companionAction,
        float componentAction,
        float componentId,
        int32_t conditions
    } rebootCmd;
} SMMessageData_t; // Messasage data

// Motor test, accel calibration, compass calibration, toggle safety switch, reboot flight controller
typedef struct SMMessage{
    enum{ // Type of data being sent to SM
        MAV_CMD_DO_MOTOR_TEST,
        MAV_CMD_PREFLIGHT_CALIBRATION, // for accel and for compass
        MAV_CMD_DO_SET_SAFETY_SWITCH_STATE, 
        MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN 
    } dataType;
    SMMessageData_t tmMessageData; // Message data
    uint32_t timeBootMs = 0; // When the message was generated
} SMMessage_t;

// pack data to send, but MAVLINK sends data as COMMAND_INT?
inline SMMessage_t motorTestPack(uint32_t time_boot_ms, float instance, float throttleType, float throttle, float timeout, int32_t motorCount, int32_t testOrder) {

}

inline SMMessage_t calibrationPack(uint32_t time_boot_ms, int32_t accelCalibration, int32_t compassCalibration) {
    
}

inline SMMessage_t setSafetySwitchPack(uint32_t time_boot_ms) {
    
}

inline SMMessage_t rebootPack(uint32_t time_boot_ms) {
    
}