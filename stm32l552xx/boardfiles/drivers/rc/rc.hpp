#pragma once

#include "rc_defines.hpp"
#include "rc_iface.hpp"
#include "stm32l5xx_hal.h"
#include "error.h"

typedef struct {
    int dataOffset;
    uint8_t mask;
    int bitshift;
} DataChunk_t;

class RCReceiver : public IRCReceiver {
    public:
        RCReceiver(UART_HandleTypeDef *uart);

        ZP_ERROR_e getRCData(RCControl *data) override;

        UART_HandleTypeDef* getHUART();

        /**
         * @brief starts DMA receive
         */
        ZP_ERROR_e init();

        /**
         * @brief restarts DMA
         */
        ZP_ERROR_e startDMA();
        /**
         * @brief Updates RCControl values
         */
        ZP_ERROR_e parse();

    private:
        UART_HandleTypeDef *uart;
        RCControl rcData;
        uint8_t rawSbus[SBUS_PACKET_SIZE];

        ZP_ERROR_e sbusToRCControl(float *value, uint8_t *buf, int channelMappingIdx);
};
