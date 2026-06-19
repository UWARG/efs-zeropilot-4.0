#include "rfd.hpp"
#include "stm32l5xx_hal_uart.h"
#include <cstring>

RFD* RFD::instance = nullptr;

RFD::RFD(UART_HandleTypeDef* huart) : huart(huart), readIndex(0), writeIndex(0) {
    instance = this;
}

RFD::~RFD() {
    instance = nullptr;
}

ZP_ERROR_e RFD::transmit(const uint8_t* data, uint16_t size) {
    if (huart) {
        if (HAL_UART_Transmit_DMA(huart, data, size) == HAL_OK) {
            return ZP_ERROR_OK;
        } else {
            return ZP_ERROR_FAIL;
        }
    }
    return ZP_ERROR_RESOURCE_UNAVAILABLE;
}

ZP_ERROR_e RFD::getRXTransferSize(uint16_t idx, uint16_t& output) {
	if (idx > lastIdx) {
		output = (uint16_t)(idx - lastIdx);
	} else {
		output = (uint16_t)(BUFFER_SIZE - lastIdx + idx);
	}
    return ZP_ERROR_OK;
}

ZP_ERROR_e RFD::init() {
    if (huart) {
        if (HAL_UARTEx_ReceiveToIdle_DMA(huart, rxBuffer, BUFFER_SIZE) == HAL_OK) {
            return ZP_ERROR_OK;
        } else {
            return ZP_ERROR_FAIL;
        }
    }
    return ZP_ERROR_RESOURCE_UNAVAILABLE;
}

ZP_ERROR_e RFD::receiveCallback(uint16_t writeIdx) {
    ZP_ERROR_e result = ZP_ERROR_OK;

    // 1. Entry Check
    if (HAL_UARTEx_GetRxEventType(huart) == HAL_UART_RXEVENT_HT) {
        result = ZP_ERROR_OK; 
    } else {
        // 2. State Calculation
        writeIndex = writeIdx % BUFFER_SIZE;
        uint16_t transferSize = 0;
        result |= getRXTransferSize(writeIndex, transferSize);

        // 3. Overflow Logic
        if ((currentSize + transferSize) > BUFFER_SIZE) {
            readIndex = (readIndex + ((currentSize + transferSize) - BUFFER_SIZE)) % BUFFER_SIZE;
            currentSize = BUFFER_SIZE;
            result = ZP_ERROR_MEMORY_OVERFLOW;
        } else {
            // 4. Standard Update
            currentSize += transferSize;
            lastIdx = writeIdx;
            result = ZP_ERROR_OK;
        }
    }

    // 5. Single Exit Point
    return result;
}

ZP_ERROR_e RFD::receive(uint8_t* buffer, uint16_t bufferSize, uint16_t &received_size) {
    if (readIndex == writeIndex) {
        return ZP_ERROR_FAIL;
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
    received_size = dataRead;
    return ZP_ERROR_OK;
}

UART_HandleTypeDef* RFD::getHUART() const {
    return huart;
}
