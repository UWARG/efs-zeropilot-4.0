#pragma once
#include "stm32h7xx_hal.h"
#include <cstdint>
#include "zp_error.h"

class UARTInterface {
    public:
        UARTInterface(UART_HandleTypeDef *huart);
        UART_HandleTypeDef* getHuart();

    private:
        UART_HandleTypeDef *huart;
        ZP_Error statusCheck(HAL_StatusTypeDef status);
        void setBit(uint32_t reg, uint32_t bit);
        ZP_Error getState();
        ZP_Error transmit(uint8_t* pData, uint16_t size);
        ZP_Error transmitDMA(uint8_t* pData, uint16_t size);
        ZP_Error receive(uint8_t *pData, uint16_t size, uint32_t deadline);
        ZP_Error restartDMA(uint8_t *pData, uint16_t size);
};