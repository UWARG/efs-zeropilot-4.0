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

ZP_Error RFD::transmit(const uint8_t* data, uint16_t size) {
    if (huart == nullptr || data == nullptr) {
        return ZP_ERROR_NULLPTR;
    }

    if (size > TX_BUFFER_SIZE) {
        return ZP_ERROR_RANGE;
    }

    if (huart->gState != HAL_UART_STATE_READY) {
        return ZP_ERROR_EXT_API | ZP_ERROR_BUSY;
    }

    memcpy(txBuffer, data, size);

    HAL_StatusTypeDef status = HAL_UART_Transmit_DMA(huart, txBuffer, size);
    if (status == HAL_BUSY) {
        return ZP_ERROR_EXT_API | ZP_ERROR_BUSY;
    } else if (status != HAL_OK) {
        return ZP_ERROR_EXT_API | ZP_ERROR_FAIL;
    }
    return ZP_ERROR_OK;
}

ZP_Error RFD::restartRx() {
    if (huart == nullptr) {
        return ZP_ERROR_NULLPTR;
    }

    HAL_StatusTypeDef status = HAL_UARTEx_ReceiveToIdle_DMA(huart, rxBuffer, BUFFER_SIZE);
    if (status == HAL_BUSY) {
        return ZP_ERROR_EXT_API | ZP_ERROR_BUSY;
    } else if (status != HAL_OK) {
        return ZP_ERROR_EXT_API | ZP_ERROR_FAIL;
    }

    readIndex = 0;
    writeIndex = 0;
    currentSize = 0;
    lastIdx = 0;

    return ZP_ERROR_OK;
}

ZP_Error RFD::getRXTransferSize(uint16_t idx, uint16_t& output) {
    output = (uint16_t)((idx + BUFFER_SIZE - lastIdx) % BUFFER_SIZE);
    return ZP_ERROR_OK;
}

ZP_Error RFD::init() {
    return restartRx();
}

ZP_Error RFD::receiveCallback(uint16_t dmaWritePos) {
    ZP_Error result = ZP_ERROR_OK;

    writeIndex = dmaWritePos % BUFFER_SIZE;
    uint16_t transferSize = 0;
    result |= getRXTransferSize(writeIndex, transferSize);

    if ((currentSize + transferSize) > RX_CAPACITY) {
        readIndex = (uint16_t)((writeIndex + BUFFER_SIZE - RX_CAPACITY) % BUFFER_SIZE);
        currentSize = RX_CAPACITY;
        result |= ZP_ERROR_MEMORY_OVERFLOW;
    } else {
        currentSize += transferSize;
    }

    lastIdx = writeIndex;

    return result;
}

ZP_Error RFD::receive(uint8_t* buffer, uint16_t bufferSize, uint16_t &received_size) {
    if (buffer == nullptr) {
        return ZP_ERROR_NULLPTR;
    }

    ZP_Error result = ZP_ERROR_OK;
    received_size = 0;

    // The DMA is not running, so reception was aborted by an error and has to be restarted
    if (huart->RxState == HAL_UART_STATE_READY) {
        return restartRx();
    }

    const uint16_t AVAILABLE = (uint16_t)((writeIndex + BUFFER_SIZE - readIndex) % BUFFER_SIZE);
    if (AVAILABLE == 0) {
        return result;
    }

    // Take what fits and leave the rest queued, rather than refusing to drain an oversized backlog
    const uint16_t TO_READ = (AVAILABLE > bufferSize) ? bufferSize : AVAILABLE;
    uint16_t firstChunk = (uint16_t)(BUFFER_SIZE - readIndex);
    if (firstChunk > TO_READ) {
        firstChunk = TO_READ;
    }

    memcpy(buffer, rxBuffer + readIndex, firstChunk);
    if (TO_READ > firstChunk) {
        memcpy(buffer + firstChunk, rxBuffer, (size_t)(TO_READ - firstChunk));
    }

    readIndex = (uint16_t)((readIndex + TO_READ) % BUFFER_SIZE);
    currentSize = (currentSize > TO_READ) ? (uint16_t)(currentSize - TO_READ) : 0;
    received_size = TO_READ;

    return result;
}

UART_HandleTypeDef* RFD::getHuart() const {
    return huart;
}
