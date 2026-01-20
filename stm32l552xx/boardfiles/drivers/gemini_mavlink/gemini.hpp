#pragma once

#include "rc_iface.hpp"
#include "rfd_iface.hpp"
#include "gemini_defines.hpp"
#include "stm32l5xx_hal.h"


class GeminiMavlink: public IRCReceiver, public IRFD {
    public:
        static GeminiMavlink* instance; // One instance only

        GeminiMavlink(UART_HandleTypeDef* huart);

        ~GeminiMavlink() override;

        // Both IRFD and IRC
        void init();

        // IRFD
        void transmit(const uint8_t* data, uint16_t size) override;
        uint16_t receive(uint8_t* buffer, uint16_t bufferSize) override;

        // IRC
        RCControl getRCData() override;
        void startDMA();

        // Helper
        UART_HandleTypeDef* getHuart() const;
        void irqhandler();

    private:
        UART_HandleTypeDef* huart;
        
        // IRFD
        uint8_t rfdRxBuffer[MAVLINK_MAX_PACKET_SIZE];
        uint16_t readIndex;
        uint16_t writeIndex;

        // RC
        RCControl rcData_;
        uint8_t rcRxBuffer[MAVLINK_MAX_PACKET_SIZE];
        uint8_t processBuffer[MAVLINK_MAX_PACKET_SIZE];

};