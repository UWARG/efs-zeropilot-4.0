#pragma once

#include "uart_iface.hpp"
#include "stm32h7xx_hal.h"
#include "stm32h7xx_hal_uart.h"

class UARTInterface : public IUARTInterface  {
    private: 
        UART_HandleTypeDef *huart;

    protected:
        UARTInterface(UART_HandleTypeDef *huart);

        // huart Getter, Setters
        UART_HandleTypeDef* getHuart();
        void setBit(volatile uint32_t &reg, uint32_t bit);

        // huart State, Error to ZPError Mapping
        ZP_Error getState();
        ZP_Error uartErrorToZpError(); 
        ZP_Error uartStatusToZpError(HAL_StatusTypeDef status);

    public:    
        // huart Register Operations
        ZP_Error transmit(uint8_t* pData, uint16_t size);
        ZP_Error transmitDMA(uint8_t* pData, uint16_t size);
        ZP_Error receive(uint8_t *pData, uint16_t size, uint32_t deadline);
        ZP_Error restartDMA(uint8_t *pData, uint16_t size);
};