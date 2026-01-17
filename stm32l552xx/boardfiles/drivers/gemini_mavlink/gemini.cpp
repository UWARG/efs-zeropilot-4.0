#include "gemini_defines.hpp"
#include "gemini.hpp"
#include <cstring>

GeminiMavlink* GeminiMavlink::instance = nullptr; // global instance

GeminiMavlink::GeminiMavlink(UART_HandleTypeDef* huart): huart(huart), readIndex(0), writeIndex(0) {
    // Clear RC buffer upon instantiation
    memset(rcRxBuffer, 0, MAVLINK_MAX_PACKET_SIZE);
    instance = this;
}

GeminiMavlink::~GeminiMavlink() {
    instance = nullptr;
}

UART_HandleTypeDef* GeminiMavlink::getHuart() const {
    return huart;
}