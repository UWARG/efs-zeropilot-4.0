#pragma once

#include "rc_defines.hpp"
#include "rc_iface.hpp"
#include "stm32h7xx_hal.h"
#include "zp_error.h"

typedef struct {
    int dataOffset;
    uint8_t mask;
    int bitshift;
} DataChunk_t;

class SBUSReceiver : public IRCReceiver {
    public:
        SBUSReceiver(UART_HandleTypeDef *uart);

        ZP_ERROR_e getRCData(RCControl &data) override;

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
        uint8_t volatile rawSbus[SBUS_PACKET_SIZE];

        ZP_ERROR_e sbusToRCControl(uint8_t *buf, int channelMappingIdx, float &output);
};
