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
        HAL_UARTEx_ReceiveToIdle_DMA(huart, rcRxBuffer, MAVLINK_MAX_PACKET_SIZE);
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