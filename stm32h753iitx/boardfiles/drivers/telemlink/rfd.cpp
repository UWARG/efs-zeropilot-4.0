#include "rfd.hpp"
#include "stm32h7xx_hal_uart.h"
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

    // Whatever was mid transfer is gone, so start the ring empty again
    readIndex = 0;
    writeIndex = 0;
    currentSize = 0;
    lastIdx = 0;

    HAL_StatusTypeDef status = HAL_UARTEx_ReceiveToIdle_DMA(huart, rxBuffer, BUFFER_SIZE);
    if (status == HAL_BUSY) {
        return ZP_ERROR_EXT_API | ZP_ERROR_BUSY;
    } else if (status != HAL_OK) {
        return ZP_ERROR_EXT_API | ZP_ERROR_FAIL;
    }
    return ZP_ERROR_OK;
}

ZP_Error RFD::getRXTransferSize(uint16_t idx, uint16_t& output) {
    if (idx > lastIdx) {
        output = (uint16_t)(idx - lastIdx);
    } else {
        output = (uint16_t)(BUFFER_SIZE - lastIdx + idx);
    }
    return ZP_ERROR_OK;
}

ZP_Error RFD::init() {
    return restartRx();
}

ZP_Error RFD::receiveCallback(uint16_t writeIdx) {
    ZP_Error result = ZP_ERROR_OK;

    if (HAL_UARTEx_GetRxEventType(huart) != HAL_UART_RXEVENT_HT) {
        writeIndex = writeIdx % BUFFER_SIZE;
        uint16_t transferSize = 0;
        result |= getRXTransferSize(writeIndex, transferSize);

        if ((currentSize + transferSize) > BUFFER_SIZE) {
            readIndex = (readIndex + ((currentSize + transferSize) - BUFFER_SIZE)) % BUFFER_SIZE;
            currentSize = BUFFER_SIZE;
            result |= ZP_ERROR_MEMORY_OVERFLOW;
        } else {
            currentSize += transferSize;
        }

        lastIdx = writeIdx;
    }

    return result;
}

ZP_Error RFD::receive(uint8_t* buffer, uint16_t bufferSize, uint16_t &received_size) {
    if (buffer == nullptr) {
        return ZP_ERROR_NULLPTR;
    }

    received_size = 0;

    // The DMA is not running, so reception was aborted by an error and has to be restarted
    if (huart->RxState == HAL_UART_STATE_READY) {
        ZP_Error restartStatus = restartRx();
        return (restartStatus == ZP_ERROR_OK) ? ZP_ERROR_NOT_READY : restartStatus;
    }

    // Nothing buffered yet is the normal idle case, not a failure
    if (readIndex == writeIndex) {
        return ZP_ERROR_NOT_READY;
    }

    int dataRead = 0;

    if (readIndex < writeIndex) {
        if ((writeIndex - readIndex) > bufferSize) {
            return ZP_ERROR_RANGE;
        }
        memcpy(buffer, rxBuffer + readIndex, writeIndex - readIndex);
        dataRead += writeIndex - readIndex;

    // data wrapped around buffer
    } else {
        if ((BUFFER_SIZE - readIndex + writeIndex) > bufferSize) {
            return ZP_ERROR_RANGE;
        }
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

UART_HandleTypeDef* RFD::getHuart() const {
    return huart;
}
