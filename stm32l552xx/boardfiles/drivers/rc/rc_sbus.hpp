#pragma once

#include "rc_defines.hpp"
#include "rc_iface.hpp"
#include "stm32l5xx_hal.h"
#include "zp_error.h"

typedef struct {
    int dataOffset;
    uint8_t mask;
    int bitshift;
} DataChunk_t;

class SBUSReceiver : public IRCReceiver {
    public:
        SBUSReceiver(UART_HandleTypeDef *uart);

        ZP_Error getRCData(RCControl &data) override;

        UART_HandleTypeDef* getHuart();

        /**
         * @brief starts DMA receive
         */
        ZP_Error init();

        /**
         * @brief restarts DMA
         */
        ZP_Error startDMA();
        /**
         * @brief Updates RCControl values
         */
        ZP_Error parse();
       
    private:
        UART_HandleTypeDef *uart;
        RCControl rcData;
        uint8_t volatile rawSbus[SBUS_PACKET_SIZE];

        ZP_Error sbusToRCControl(uint8_t *buf, int channelMappingIdx, float &output);
};
