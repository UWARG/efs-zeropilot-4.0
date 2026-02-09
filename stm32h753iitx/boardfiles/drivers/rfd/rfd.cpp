#include "rfd.hpp"
#include "stm32h7xx_hal_uart.h"
#include <cstring>

RFD* RFD::instance = nullptr;

RFD::RFD(UART_HandleTypeDef* huart) : huart(huart), readIndex(0), writeIndex(0){
    instance = this;
}

RFD::~RFD() {
    instance = nullptr;
}

void RFD::transmit(const uint8_t* data, uint16_t size) {
    if (huart) {
        HAL_UART_Transmit_DMA(huart, data, size);
    }
}

uint16_t RFD::getRXTransferSize(uint16_t idx) {
	if (idx > lastIdx) {
		return (uint16_t)(idx - lastIdx);
	} else {
		return (uint16_t)(BUFFER_SIZE - lastIdx + idx);
	}
}

void RFD::startReceive() {
    if (huart) {
        HAL_UARTEx_ReceiveToIdle_DMA(huart, rxBuffer, BUFFER_SIZE);
    }
}

void RFD::receiveCallback(uint16_t writeIdx) {
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

uint16_t RFD::receive(uint8_t* buffer, uint16_t bufferSize) {
    if (readIndex == writeIndex) {
        return 0;
    }

    int dataRead = 0;

	if (readIndex < writeIndex) {
		memcpy(buffer, rxBuffer + readIndex, writeIndex - readIndex);
		dataRead += writeIndex - readIndex;
    
    // data wrapped around buffer
	} else {
		memcpy(buffer, rxBuffer + readIndex, BUFFER_SIZE - readIndex);
		dataRead += BUFFER_SIZE - readIndex;

		memcpy(buffer + dataRead, rxBuffer, writeIndex);
		dataRead += writeIndex;
	}

    readIndex = (readIndex + dataRead) % BUFFER_SIZE;
    currentSize -= dataRead;
    return dataRead;
}

UART_HandleTypeDef* RFD::getHuart() const {
    return huart;
}
