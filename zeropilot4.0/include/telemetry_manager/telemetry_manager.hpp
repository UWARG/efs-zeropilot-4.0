#pragma once

#define TM_SCHEDULING_RATE_HZ 20
#define TM_UPDATE_LOOP_DELAY_MS (1000 / TM_SCHEDULING_RATE_HZ)

#define MAVLINK_MSG_MAX_SIZE 280
#define RX_BUFFER_LEN 8192

#define TM_LINK_BAUDRATE 57600
#define TM_LINK_TX_LOADING_FACTOR 0.8f

#define TM_MAX_TRANSCEIVE (uint16_t) ((TM_LINK_BAUDRATE / (8 * TM_SCHEDULING_RATE_HZ)))
#define TM_MAX_TX_BYTES (uint16_t) (TM_LINK_TX_LOADING_FACTOR * TM_MAX_TRANSCEIVE)
#define TM_MAX_RX_BYTES (uint16_t) (TM_MAX_TRANSCEIVE + MAVLINK_MSG_MAX_SIZE)

#include "systemutils_iface.hpp"
#include "mavlink.h"
#include "queue_iface.hpp"
#include "tm_queue.hpp"
#include "rc_motor_control.hpp"
#include "telemlink_iface.hpp"
#include "tm_param_setup.hpp"

struct rtcm_correction_data_t {
  uint8_t data[720];
  uint16_t len;
  bool newData;
};
extern rtcm_correction_data_t sharedRtcmBuffer;

class TelemetryManager {
    friend class TMParamSetup;

  private:
    ISystemUtils *systemUtilsDriver;                        // System Utils Driver
    ITelemLink *telemLinkDriver;                            // Driver used to actually send mavlink messages
    IMessageQueue<TMMessage_t> *tmTXQueueDriver;            // Driver that receives messages from other managers
    IMessageQueue<RCMotorControlMessage_t> *amQueueDriver;   // Driver that currently is only used to set arm/disarm
    IMessageQueue<mavlink_message_t> *packedMsgBuffer{};    // GPOS, Attitude and Heartbeat/Connection Messages
    mavlink_status_t status;
    mavlink_message_t overflowBuf;
    bool overflowMsgPending;

    uint16_t currParamListTxIdx;

    uint8_t txBuffer[TM_MAX_TX_BYTES];
    uint8_t rxBuffer[TM_MAX_RX_BYTES];

    // rtcm
    uint8_t rtcmAssemblyBuffer[720]; // 180 * 4(Max Frag Count)
    uint8_t rtcmLen;
    uint8_t rtcmCurrentSequenceId;
    uint8_t rtcmRecievedFragments; // bit n set to 1 means fragment n has been recieved. Other non-related bits(Other than 4 LSB) are set to 0

    void processRxMsg(const mavlink_message_t &msg);
    void processTXMsgQueue();
    void transmit();
    void receive();
    void processParamTx();
    void enqueueParamValueTx(uint16_t index);
    void handleRtcmFragment(const mavlink_gps_rtcm_data_t &rtcmMsg);
    void resetRtcmState();

    uint8_t profilerId;
    
  public:
    TelemetryManager(ISystemUtils *systemUtilsDriver, ITelemLink *telemLinkDriver, IMessageQueue<TMMessage_t>  *tmTXQueueDriver,  IMessageQueue<RCMotorControlMessage_t> *amQueueDriver,IMessageQueue<mavlink_message_t> *packedMsgBuffer);
    ~TelemetryManager();

    void tmUpdate();

    TMParamSetup paramSetup;
};
