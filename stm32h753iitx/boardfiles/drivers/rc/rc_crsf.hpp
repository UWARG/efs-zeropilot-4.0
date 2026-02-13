#pragma once

#include "rc_defines.hpp"
#include "rc_iface.hpp"
#include "stm32h7xx_hal.h"
/**
 * @class CRSFReceiver
 * @brief A class to receive and parse CRSF RC channel data via UART.
 *
 * @note UART Configuration Requirements:
 * - BaudRate: 420000
 * - WordLength: UART_WORDLENGTH_8B
 * - Parity: UART_PARITY_NONE
 * - StopBits: UART_STOPBITS_1
 * - Mode: UART_MODE_RX
 * - HwFlowCtl: UART_HWCONTROL_NONE
 * - OverSampling: UART_OVERSAMPLING_16
 * - OneBitSampling: UART_ONE_BIT_SAMPLE_DISABLE
 * - ClockPrescaler: UART_PRESCALER_DIV1
 * - AdvancedInit.AdvFeatureInit: UART_ADVFEATURE_NO_INIT
 */
class CRSFReceiver : public IRCReceiver {
    public:
        CRSFReceiver(UART_HandleTypeDef *uart);

        RCControl getRCData() override;

        void init();
        void startDMA();

        void parse();

        UART_HandleTypeDef * getHUART();
        
    private:
        UART_HandleTypeDef *uart_;
        RCControl rcData_;
        uint8_t crsfRxBuffer_[CRSF_PACKET_SIZE];
};