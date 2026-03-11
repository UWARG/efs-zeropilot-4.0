#pragma once
#define MAVLINK_MSG_MAX_SIZE 280
#define MAVLINK_MAX_IDENTIFIER_LEN 17
#define RX_BUFFER_LEN 8192

#include "systemutils_iface.hpp"
#include "mavlink.h"
#include "queue_iface.hpp"
#include "tm_queue.hpp"
#include "rc_motor_control.hpp"
#include "rfd_iface.hpp"
class TelemetryManager {
  private:
    ISystemUtils *systemUtilsDriver;			                  // System Utils Driver
    IRFD *rfdDriver;										                    // Driver used to actually send mavlink messages
    IMessageQueue<TMMessage_t> *tmQueueDriver;				      // Driver that receives messages from other managers
    IMessageQueue<RCMotorControlMessage_t> *amQueueDriver;	// Driver that currently is only used to set arm/disarm
    IMessageQueue<mavlink_message_t> *messageBuffer{};	  	// GPOS, Attitude and Heartbeat/Connection Messages
    mavlink_status_t status;
    mavlink_message_t message;
    mavlink_message_t overflowBuf;
    bool isInitialized = false;
    bool overflowMsgPending = false;

    ZP_ERROR_e handleRxMsg(const mavlink_message_t &msg);
    ZP_ERROR_e processMsgQueue();
    ZP_ERROR_e transmit();
    ZP_ERROR_e reconstructMsg();

  public:
    TelemetryManager(ISystemUtils *systemUtilsDriver, IRFD *rfdDriver, IMessageQueue<TMMessage_t>  *tmQueueDriver,  IMessageQueue<RCMotorControlMessage_t> *amQueueDriver,IMessageQueue<mavlink_message_t> *messageBuffer);
    ~TelemetryManager();
    
    ZP_ERROR_e init() {
      if (isInitialized) return ZP_ERROR_ALREADY_INITIALIZED;
      if (!systemUtilsDriver || !rfdDriver || 
          !tmQueueDriver || !amQueueDriver || !messageBuffer) {
          return ZP_ERROR_NULLPTR;
      }
      isInitialized = true;
      return ZP_ERROR_OK;
    }

    ZP_ERROR_e tmUpdate();
};
