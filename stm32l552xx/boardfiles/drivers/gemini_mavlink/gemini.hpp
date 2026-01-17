#pragma once

#include "rc_iface.hpp"
#include "rfd_iface.hpp"
#include "gemini_defines.hpp"
#include "stm32l5xx_hal.h"


class GeminiMavlink: public IRCReceiver, public IRFD {
    public:
        GeminiMavlink(UART_HandleTypeDef* huart);

        ~GeminiMavlink() override;

        // IRFD
        void transmit(const uint8_t* data, uint16_t size) override;
        uint16_t receive(uint8_t* buffer, uint16_t bufferSize) override;

        // IRC
        RCControl getRCData() override;

    private:
        UART_HandleTypeDef* huart;
        
        // IRFD
        uint8_t rxBuffer[MAVLINK_MAX_PACKET_SIZE]; // think of better name (to not confuse with other buffer)
        uint16_t readIndex;
        uint16_t writeIndex;

        // RC
        RCControl rcData_;
        uint8_t mavlinkRxBuffer[MAVLINK_MAX_PACKET_SIZE]; // think of better name (to not confuse with other buffer)

};