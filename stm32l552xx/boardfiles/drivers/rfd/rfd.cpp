#include "rfd.hpp"
#include "stm32l5xx_hal_uart.h"

/**
 * Important consideraton when using this driver:
 * Recieve should be called fast enough to never let the buffer overflow,
 * since there is currently no way to recover from an overflow
 */

RFD* RFD::instance = nullptr; // global instance

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

void RFD::startReceive() {
    if (huart) {
        HAL_UARTEx_ReceiveToIdle_DMA(huart, rxBuffer, BUFFER_SIZE);
    }
}

uint16_t RFD::receive(uint8_t* buffer, uint16_t bufferSize) {
    for (uint16_t i = 0; i < bufferSize; i++) {
        if (readIndex == writeIndex){
            return i;
        }
        buffer[i] = rxBuffer[readIndex];
        readIndex++;
        if (readIndex >= BUFFER_SIZE) {
            readIndex = 0;
        }
    }
    return bufferSize;
}

void RFD::receiveCallback(uint16_t size){
    writeIndex = size;
	HAL_UARTEx_ReceiveToIdle_DMA(huart, rxBuffer, BUFFER_SIZE);
}

UART_HandleTypeDef* RFD::getHuart() const {
    return huart;
}
