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
    IMessageQueue<TMMessage_t> *tmRxQueue;				      // Driver that receives messages from other managers
    IMessageQueue<RCMotorControlMessage_t> *amQueueDriver;	// Driver that currently is only used to set arm/disarm
    IMessageQueue<mavlink_message_t> *mavlinkTxQueue{};	  	// GPOS, Attitude, Heartbeat/Connection, and Param Messages
    IMessageQueue<TMSMMessage_t> *tmTxQueue;            // Driver that sends messages to system manager
    mavlink_status_t status;
    mavlink_message_t message;
    mavlink_message_t overflowBuf;
    bool overflowMsgPending;
    bool transmittingParams = false; // Flag will be set when TM receives a param list request and unset when finished

    void handleRxMsg(const mavlink_message_t &msg);
    void processMsgQueue();
    void transmit();
    void reconstructMsg();

  public:
    TelemetryManager(ISystemUtils *systemUtilsDriver, IRFD *rfdDriver, IMessageQueue<TMMessage_t>  *tmRxQueue, IMessageQueue<TMSMMessage_t> *tmTxQueue, IMessageQueue<RCMotorControlMessage_t> *amQueueDriver,IMessageQueue<mavlink_message_t> *mavlinkTxQueue);
    ~TelemetryManager();

    void tmUpdate();
};
