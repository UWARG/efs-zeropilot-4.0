#pragma once

#include "systemutils.hpp"
#include "iwdg.hpp"
#include "logger.hpp"
#include "motor.hpp"
#include "motor_datatype.hpp"
#include "rc_sbus.hpp"
#include "rc_crsf.hpp"
#include "rc_motor_control.hpp"
#include "tm_queue.hpp"
#include "mavlink.h"
#include "queue.hpp"
#include "gps.hpp"
#include "rfd.hpp"
#include "imu.hpp"
#include "power_module.hpp"
#include "baromter.hpp"

extern SystemUtils *systemUtilsHandle;

extern IndependentWatchdog *iwdgHandle;
extern Logger *loggerHandle;

extern MotorControl *motor1Handle;
extern MotorControl *motor2Handle;
extern MotorControl *motor3Handle;
extern MotorControl *motor4Handle;
extern MotorControl *motor5Handle;
extern MotorControl *motor6Handle;
extern MotorControl *motor7Handle;
extern MotorControl *motor8Handle;

extern CRSFReceiver *rcHandle;
extern GPS *gpsHandle;
extern RFD *telemLinkHandle;
extern IMU *imuHandle;
extern PowerModule *pmHandle;
extern Barometer *barometerHandle;

extern MessageQueue<RCMotorControlMessage_t> *amRCQueueHandle;
extern MessageQueue<char[100]> *smLoggerQueueHandle;
extern MessageQueue<TMMessage_t> *tmQueueHandle;
extern MessageQueue<mavlink_message_t> *messageBufferHandle;

extern MotorGroupInstance_t mainMotorGroup;

void initDrivers();
