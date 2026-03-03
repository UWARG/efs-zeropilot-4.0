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

        // Both RFD and RC
        void init();

        // RFD
        void transmit(const uint8_t* data, uint16_t size) override; // similar to rfd
        uint16_t receive(uint8_t* buffer, uint16_t bufferSize) override; // identical (almost) to rfd

        // RC
        RCControl getRCData() override;

        void forcePushMAVLinkRC(RCControl rcData) override;

        // Helper
        UART_HandleTypeDef* getHuart() const;
        void irqhandler(uint16_t size); // in override.cpp check that out ()
        

    private:
        UART_HandleTypeDef* huart;
        
        // IRFD
        uint8_t rfdRxBuffer[MAVLINK_MAX_PACKET_SIZE];
        uint16_t readIndex;
        uint16_t writeIndex;

        // RC
        RCControl rcData_;

};