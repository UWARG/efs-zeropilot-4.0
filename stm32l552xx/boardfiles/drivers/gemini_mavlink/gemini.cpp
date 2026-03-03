#include "gemini_defines.hpp"
#include "gemini.hpp"
#include <cstring>
#include "mavlink.h"

GeminiMavlink* GeminiMavlink::instance = nullptr; // global instance

GeminiMavlink::GeminiMavlink(UART_HandleTypeDef* huart): huart(huart), readIndex(0), writeIndex(0) {
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
    if(huart) {
        HAL_UART_Transmit_DMA(huart, data, size);
    }
}

uint16_t GeminiMavlink::receive(uint8_t* buffer, uint16_t bufferSize) {
    for(uint16_t i = 0; i < bufferSize; i++) {
        if(readIndex == writeIndex) {
            return i;
        }
        buffer[i] = rfdRxBuffer[readIndex];
        readIndex++;
        if(readIndex >= MAVLINK_MAX_PACKET_SIZE) {
            readIndex = 0;
        }
    }
    return bufferSize;
}

RCControl GeminiMavlink::getRCData() {
    RCControl tmp;
    tmp.isDataNew = false;
    return tmp;
}

UART_HandleTypeDef* GeminiMavlink::getHuart() const {
    return huart;
}

void GeminiMavlink::irqhandler(uint16_t size) {
    writeIndex = size;
}

void GeminiMavlink::forcePushMAVLinkRC(RCControl rcData) {
    rcData_ = rcData;
}