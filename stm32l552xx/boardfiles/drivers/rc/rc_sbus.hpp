#pragma once

#include "rc_defines.hpp"
#include "rc_iface.hpp"
#include "stm32l5xx_hal.h"

typedef struct {
    int dataOffset;
    uint8_t mask;
    int bitshift;
} DataChunk_t;

class SBUSReceiver : public IRCReceiver {
    public:
        SBUSReceiver(UART_HandleTypeDef *uart);

        RCControl getRCData() override;

        UART_HandleTypeDef* getHUART();

        /**
         * @brief starts DMA receive
         */
        void init();

        /**
         * @brief restarts DMA
         */
        void startDMA();
        /**
         * @brief Updates RCControl values
         */
        void parse();

        /**
         * @brief Returns the UART handle used by the SBUS receiver
         */
        UART_HandleTypeDef * getUart();
       
    private:
        UART_HandleTypeDef *uart;
        RCControl rcData;
        uint8_t rawSbus[SBUS_PACKET_SIZE];

        float sbusToRCControl(uint8_t *buf, int channelMappingIdx);
};
