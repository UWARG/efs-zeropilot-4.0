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
        bool accelCalibration,
        bool compassCalibration
    } calibrationCmd;
    struct {
        float switchState
    } setSafetySwitchCmd;
    struct {
        bool reboot
    } rebootCmd;
} SMMessageData_t; // Messasage data

// Motor test, accel calibration, compass calibration, toggle safety switch, reboot flight controller
typedef struct SMMessage{
    enum{ // Type of data being sent to SM
        MOTOR_TEST,
        CALIBRATION, // For acceleration and for compass
        SAFETY_SWITCH_STATE, 
        REBOOT_FLIGHT_CONTROLLER
    } dataType;
    SMMessageData_t tmMessageData; // Message data
    uint32_t timeBootMs = 0; // When the message was generated
} SMMessage_t;

inline SMMessage_t motorTestPack(uint32_t time_boot_ms, float instance, float throttleType, float throttle, float timeout, int32_t motorCount, int32_t testOrder) {
    const TMMessageData_t DATA = {.motorTestCmd={instance, throttleType, throttle, timeout, motorCount, testOrder}};
    return TMMessage_t{TMMessage_t::MOTOR_TEST, DATA, time_boot_ms};
}

inline SMMessage_t calibrationPack(uint32_t time_boot_ms, bool accelCalibration, bool compassCalibration) {
    const TMMessageData_t DATA = {.motorTestCmd={accelCalibration, compassCalibration}};
    return TMMessage_t{TMMessage_t::CALIBRATION, DATA, time_boot_ms};
}

inline SMMessage_t setSafetySwitchPack(uint32_t time_boot_ms, float switchState) {
    const TMMessageData_t DATA = {.motorTestCmd={switchState}};
    return TMMessage_t{TMMessage_t::SAFETY_SWITCH_STATE, DATA, time_boot_ms};
}

inline SMMessage_t rebootPack(uint32_t time_boot_ms, bool reboot) {
    const TMMessageData_t DATA = {.rebootCmd={reboot}};
    return TMMessage_t{TMMessage_t::REBOOT_FLIGHT_CONTROLLER, DATA, time_boot_ms};
}