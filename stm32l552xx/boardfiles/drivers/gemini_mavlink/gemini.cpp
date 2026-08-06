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




uint16_t GeminiMavlink::getRXTransferSize(uint16_t idx) {
	if (idx > lastIdx) {
		return (uint16_t)(idx - lastIdx);
	} else {
		return (uint16_t)(BUFFER_SIZE - lastIdx + idx);
	}
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


void GeminiMavlink::receiveCallback(uint16_t writeIdx) {
    if (HAL_UARTEx_GetRxEventType(huart) == HAL_UART_RXEVENT_HT) {
		return;
	}

    writeIndex = writeIdx % BUFFER_SIZE;

	uint16_t transferSize = getRXTransferSize(writeIndex);
	currentSize += transferSize;

    if (currentSize > (BUFFER_SIZE - 1)) {
        readIndex += currentSize - (BUFFER_SIZE - 1);
        readIndex %= BUFFER_SIZE;
        currentSize = BUFFER_SIZE - 1;
    }

	lastIdx = writeIdx;
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




void GeminiMavlink::startDMA() {
    // start circular DMA
    HAL_UARTEx_ReceiveToIdle_DMA(huart, crsfRxBuffer, CRSF_BYTE_COUNT);
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
