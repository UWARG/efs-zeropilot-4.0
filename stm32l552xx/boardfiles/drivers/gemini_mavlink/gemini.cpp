#include "gemini_defines.hpp"
#include "gemini.hpp"
#include <cstring>
#include "mavlink.h"

GeminiMavlink* GeminiMavlink::instance = nullptr; // global instance

GeminiMavlink::GeminiMavlink(UART_HandleTypeDef* huart): huart(huart), readIndex(0), writeIndex(0) {
    // Clear RC buffer upon instantiation
    memset(rcRxBuffer, 0, MAVLINK_MAX_PACKET_SIZE);
    // Create only one instance of GeminiMavlink
    instance = this;
}

GeminiMavlink::~GeminiMavlink() {
    instance = nullptr;
}

void GeminiMavlink::init() {
    if(huart) {
        // Initialize DMA buffer for RC
        rcData_.isDataNew = false;
        HAL_UARTEx_ReceiveToIdle_DMA(huart, rcRxBuffer, MAVLINK_MAX_PACKET_SIZE); // to delete (see rx and tx logic for rfd->to implement for rc) because no more circular dma
        // Initialize DMA buffer for RFD
        HAL_UARTEx_ReceiveToIdle_DMA(huart, rfdRxBuffer, MAVLINK_MAX_PACKET_SIZE);
    }
}

void GeminiMavlink::startDMA() {
    if(huart) {
        HAL_UARTEx_ReceiveToIdle_DMA(huart, rcRxBuffer, MAVLINK_MAX_PACKET_SIZE);
    }
}

void GeminiMavlink::irqhandler() {
    memcpy(processBuffer, rcRxBuffer, MAVLINK_MAX_PACKET_SIZE);
}

RCControl GeminiMavlink::getRCData() {
    
    // RC CHANNEL RAW convert to RC control struct

    // Struct to fill
    RCControl data;

    // Data is not new by default
    data.isDataNew = false;

    // Mavlink
    mavlink_message_t msg;
    mavlink_status_t status;

    // Saves copy of current data in processBuffer
    uint8_t tmp[MAVLINK_MAX_PACKET_SIZE];
    memcpy(tmp, processBuffer, MAVLINK_MAX_PACKET_SIZE);

    // Parsing
    for(uint16_t i = 0; i < MAVLINK_MAX_PACKET_SIZE; ++i) {

        if(mavlink_parse_char(MAVLINK_CHANNEL, tmp[i], &msg, &status)){
            switch(msg.msgid) {
                case MAVLINK_MSG_ID_RC_CHANNELS_OVERRIDE:
                    {
                        mavlink_rc_channels_override_t rc_raw;
                        mavlink_msg_rc_channels_override_decode(&msg, &rc_raw);

                        // Mapping (ask about throttle)
                        data.roll = (float)((rc_raw.chan1_raw - 1000.0) / 10.0f);
                        data.pitch = (float)((rc_raw.chan2_raw - 1000.0) / 10.0f);
                        data.throttle = (float)((rc_raw.chan3_raw - 1000.0) / 10.0f);
                        data.yaw = (float)((rc_raw.chan4_raw - 1000.0) / 10.0f);

                        // Arm TBD
                        
                        data.isDataNew = true;
                        
                    }
                    break;
                default:
                    break;
            }
        }
        
    }

    // Return struct
    return data;

}



UART_HandleTypeDef* GeminiMavlink::getHuart() const {
    return huart;
}

// add to rffdiface to forcerxbuffer take mavlink rcmessage and repack into sbus or crsfbuffer
// if mavlink type is rcchannels then add a case

// mavlink buffer of type rc control by 

// void forcePushMavlink(blablabla) to convert mavlink to crsf + sbus

// struct with:
// chan1 to 8 raw, timebootms

// questions:
// is forcepushmavlink to ensure backward compatibility so that the mavlink is converted into crsf / sbus depending on what driver is being used?
// takes an RCControl struct and then changes it into rawSbus, rcCrsf or GeminiMavlink (store as is)
// instead of rawsbus RC
// forcePushRC inside rc_iface.hpp just do nothing for now but put in the iface

// mavlink packets ota -> dma buffer (rfd) -> processqueue rx message push into rc buffer -> sets write flag as false (SM calls getRC)