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

ZP_ERROR_e RFD::transmit(const uint8_t* data, uint16_t size) {
    // Check for null pointers
    if (data == nullptr) {
        return ZP_ERROR_NULLPTR;
    }

    if (huart == nullptr) {
        return ZP_ERROR_NOT_READY;
    }

    // Call HAL function and check return status
    HAL_StatusTypeDef status = HAL_UART_Transmit_DMA(huart, data, size);

    switch (status) {
        case HAL_OK:
            return ZP_ERROR_OK;
        case HAL_BUSY:
            return ZP_ERROR_BUSY;
        case HAL_TIMEOUT:
            return ZP_ERROR_TIMEOUT;
        case HAL_ERROR:
        default:
            return ZP_ERROR_FAIL;
    }
}

ZP_ERROR_e RFD::startReceive() {
    if (huart == nullptr) {
        return ZP_ERROR_NOT_READY;
    }

    // Call HAL function and check return status
    HAL_StatusTypeDef status = HAL_UARTEx_ReceiveToIdle_DMA(huart, rxBuffer, BUFFER_SIZE);

    switch (status) {
        case HAL_OK:
            return ZP_ERROR_OK;
        case HAL_BUSY:
            return ZP_ERROR_BUSY;
        case HAL_TIMEOUT:
            return ZP_ERROR_TIMEOUT;
        case HAL_ERROR:
        default:
            return ZP_ERROR_FAIL;
    }
}

ZP_ERROR_e RFD::receive(uint16_t *received_size, uint8_t* buffer, uint16_t bufferSize) {
    // Check for null pointers
    if (received_size == nullptr) {
        return ZP_ERROR_NULLPTR;
    }

    if (buffer == nullptr) {
        return ZP_ERROR_NULLPTR;
    }

    // Initialize received_size
    *received_size = 0;

    // Copy data from circular buffer to output buffer
    for (uint16_t i = 0; i < bufferSize; i++) {
        if (readIndex == writeIndex){
            *received_size = i;
            return ZP_ERROR_OK;
        }
        buffer[i] = rxBuffer[readIndex];
        readIndex++;
        if (readIndex >= BUFFER_SIZE) {
            readIndex = 0;
        }
    }

    *received_size = bufferSize;
    return ZP_ERROR_OK;
}

void RFD::receiveCallback(uint16_t size){
    writeIndex = size;
}

UART_HandleTypeDef* RFD::getHuart() const {
    return huart;
}
