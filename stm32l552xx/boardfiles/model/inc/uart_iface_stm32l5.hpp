#pragma once

#include "uart_iface.hpp"
#include "stm32l5xx_hal.h"
#include "stm32l5xx_hal_uart.h"

class UARTInterface : public IUARTInterface  {
    private: 
        UART_HandleTypeDef *huart;

    protected:
        // huart Error to ZPError Mapping
        ZP_Error uartErrorToZpError(); 
        ZP_Error uartStatusToZpError(HAL_StatusTypeDef status);

    public:
        UARTInterface(UART_HandleTypeDef *huart);

        // huart Getter, Setters
        UART_HandleTypeDef* getHuart();
        void setBit(volatile uint32_t &reg, uint32_t bit);
        UART_HandleTypeDef getTXState();
        UART_HandleTypeDef getRXState();

        // huart Register Operations
        ZP_Error transmit(uint8_t* pData, uint16_t size);
        ZP_Error transmitDMA(uint8_t* pData, uint16_t size);
        ZP_Error receive(uint8_t *pData, uint16_t size, uint32_t deadline);
        ZP_Error restartDMA(uint8_t *pData, uint16_t size);
};