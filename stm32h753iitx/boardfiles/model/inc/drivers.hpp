#pragma once

#include "systemutils.hpp"
#include "iwdg.hpp"
#include "logger.hpp"
#include "sd.hpp"
#include "fs_backend.hpp"
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
#include "dshot.hpp"

extern SystemUtils *systemUtilsHandle;

extern IndependentWatchdog *iwdgHandle;

extern IMotorControl *motorHandles[8];

extern CRSFReceiver *rcHandle;
extern GPS *gpsHandle;
extern IMU *imuHandle;
extern RFD *telemLinkHandle;
extern PowerModule *pmHandle;

extern SDFileSystem *sdFileSystemHandle;
extern FatFsBackend *fatFsBackendHandle;

extern MessageQueue<RCMotorControlMessage_t> *amRCQueueHandle;
extern MessageQueue<char[100]> *smLoggerQueueHandle;
extern MessageQueue<TMMessage_t> *tmQueueHandle;
extern MessageQueue<mavlink_message_t> *messageBufferHandle;

extern MessageQueue<ExMemReqMsg> *sdRequestQueueHandle;
extern MessageQueue<ExMemReqBuff> *sdBufferQueueHandle;
extern IMessageQueue<PollResult> *sdResponseQueuesHandle[static_cast<size_t>(ManId_e::COUNT)];

extern MotorGroupInstance_t mainMotorGroup;

void initDrivers();
