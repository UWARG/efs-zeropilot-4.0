#pragma once

#include "rc_iface.hpp"
#include "telemlink_iface.hpp"
#include "gemini_defines.hpp"
#include "stm32l5xx_hal.h"


class GeminiMavlink: public IRCReceiver, public ITelemLink {
    public:
        static GeminiMavlink* instance; // One instance only

        GeminiMavlink(UART_HandleTypeDef* huart);

        ~GeminiMavlink() override;

        // Both RFD and RC
        void init();

        // RFD
        void transmit(const uint8_t* data, uint16_t size) override; // similar to rfd
        uint16_t receive(uint8_t* buffer, uint16_t bufferSize) override; // identical (almost) to rfd
        void receiveCallback(uint16_t size); // DMA callback


        // RC
        RCControl getRCData() override;
        void parse();
        void startDMA();

        void forcePushMAVLinkRC(RCControl rcData) override;

        // Helper
        UART_HandleTypeDef* getHuart() const;
        void irqhandler(uint16_t size); // in override.cpp check that out ()
        

    private:

        // IRFD
        UART_HandleTypeDef* huart;
        
        uint8_t rfdRxBuffer[MAVLINK_MAX_PACKET_SIZE];
        uint16_t getRXTransferSize(uint16_t idx);

        uint8_t rxBuffer[BUFFER_SIZE];

        uint16_t readIndex = 0;
        uint16_t writeIndex = 0;

        uint16_t currentSize = 0;
        uint16_t lastIdx = 0;



        // RC
        RCControl rcData_;
        uint8_t crsfRxBuffer[CRSF_BYTE_COUNT];


};
