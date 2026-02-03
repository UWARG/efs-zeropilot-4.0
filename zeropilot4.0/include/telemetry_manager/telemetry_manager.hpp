#pragma once
#define MAVLINK_MSG_MAX_SIZE 280
#define MAVLINK_MAX_IDENTIFIER_LEN 17
#define RX_BUFFER_LEN 8192

#define TM_RFD_BAUDRATE 57600
#define TM_SCHEDULING_RATE_HZ 20
#define TM_RFD_TX_LOADING_FACTOR 0.8f

#define TM_MAX_TRANSCEIVE (uint16_t) ((TM_RFD_BAUDRATE / (8 * TM_SCHEDULING_RATE_HZ)))
#define TM_MAX_TX_BYTES (uint16_t) (TM_RFD_TX_LOADING_FACTOR * TM_MAX_TRANSCEIVE)
#define TM_MAX_RX_BYTES (uint16_t) (TM_MAX_TRANSCEIVE + MAVLINK_MSG_MAX_SIZE)

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
    IMessageQueue<TMMessage_t> *tmTXQueueDriver;				      // Driver that receives messages from other managers
    IMessageQueue<RCMotorControlMessage_t> *amQueueDriver;	// Driver that currently is only used to set arm/disarm
    IMessageQueue<mavlink_message_t> *packedMsgBuffer{};	  	// GPOS, Attitude and Heartbeat/Connection Messages
    mavlink_status_t status;
    mavlink_message_t message;
    mavlink_message_t overflowBuf;
    bool overflowMsgPending;

    uint8_t txBuffer[TM_MAX_TX_BYTES];
    uint8_t rxBuffer[TM_MAX_RX_BYTES];

    void processRxMsg(const mavlink_message_t &msg);
    void processTXMsgQueue();
    void transmit();
    void receive();

  public:
    TelemetryManager(ISystemUtils *systemUtilsDriver, IRFD *rfdDriver, IMessageQueue<TMMessage_t>  *tmTXQueueDriver,  IMessageQueue<RCMotorControlMessage_t> *amQueueDriver,IMessageQueue<mavlink_message_t> *packedMsgBuffer);
    ~TelemetryManager();

    void tmUpdate();
};
