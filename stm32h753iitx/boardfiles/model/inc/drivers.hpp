#pragma once

#include "systemutils.hpp"
#include "mathutils.hpp"
#include "iwdg.hpp"
#include "logger.hpp"
#include "sd.hpp"
#include "motor.hpp"
#include "motor_datatype.hpp"
#include "rc_sbus.hpp"
#include "rc_crsf.hpp"
#include "rc_motor_control.hpp"
#include "tm_queue.hpp"
#include "mavlink.h"
#include "queue.hpp"
#include "gps.hpp"
#include "can_controller.hpp"
#include "rfd.hpp"
#include "imu.hpp"
#include "power_module.hpp"
#include "dshot.hpp"
#include "fused_imu.hpp"
#include "fft.hpp"
#include "tf02pro.hpp"
#include "icp_20100.hpp"

extern SystemUtils *systemUtilsHandle;
extern MathUtils *mathUtilsHandle;
extern FFT *fftHandle;

extern IndependentWatchdog *iwdgHandle;

extern IMotorControl *motorHandles[8];

extern CANController *canControllerHandle;
extern CRSFReceiver *rcHandle;
extern GPS *gpsHandle;
extern FusedIMU *imuHandle;
extern RFD *telemLinkHandle;
extern PowerModule *pmHandle;
extern Rangefinder *rangefinderHandle;
extern Barometer *barometerHandle;

extern SDFileSystem *sdFileSystemHandle;

extern MessageQueue<RCMotorControlMessage_t> *amRCQueueHandle;
extern MessageQueue<char[100]> *smLoggerQueueHandle;
extern MessageQueue<TMMessage_t> *tmQueueHandle;
extern MessageQueue<mavlink_message_t> *messageBufferHandle;

extern MessageQueue<SdReqMsg> *sdRequestQueueHandle;
extern MessageQueue<SdReqBuf> *sdBufferQueueHandle;
extern IMessageQueue<PollResult> *sdResponseQueuesHandle[static_cast<size_t>(ManagerId_e::NUM_MANAGERS)];

extern MotorGroupInstance_t mainMotorGroup;

void initDrivers();
