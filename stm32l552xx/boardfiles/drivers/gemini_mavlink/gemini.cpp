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
        HAL_UARTEx_ReceiveToIdle_DMA(huart, rfdRxBuffer, MAVLINK_MAX_PACKET_SIZE);
    }
}

void GeminiMavlink::transmit(const uint8_t* data, uint16_t size) {


    return;

}

uint16_t GeminiMavlink::receive(uint8_t* buffer, uint16_t bufferSize) {
    

    
    return 0;
}

RCControl GeminiMavlink::getRCData() {
    
    RCControl tmp;
    tmp.isDataNew = false;
    return tmp;

}

UART_HandleTypeDef* GeminiMavlink::getHuart() const {
    return huart;
}

void GeminiMavlink::startDMA() {
    if(huart) {
        HAL_UARTEx_ReceiveToIdle_DMA(huart, processBuffer, MAVLINK_MAX_PACKET_SIZE);
    }
}

void GeminiMavlink::irqhandler() {
    memcpy(processBuffer, rcRxBuffer, MAVLINK_MAX_PACKET_SIZE);
}

void GeminiMavlink::forcePushMAVLinkRC(RCControl rcData) {
    rcData_ = rcData;
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