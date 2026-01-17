#include "gemini_defines.hpp"
#include "gemini.hpp"
#include <cstring>

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

UART_HandleTypeDef* GeminiMavlink::getHuart() const {
    return huart;
}