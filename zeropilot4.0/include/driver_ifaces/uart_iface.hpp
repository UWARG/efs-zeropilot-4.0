#pragma once
#if defined(STM32L552xx)
#include "stm32l5xx_hal.h"
#elif defined(STM32H753xx)
#include "stm32h7xx_hal.h"
#else
#error "Define the target STM32 family before including uart_iface.hpp"
#endif
#include <cstdint>
#include "zp_error.h"

class UARTInterface {
    public:
        UARTInterface(UART_HandleTypeDef *hxuart);
        UART_HandleTypeDef* getHuart();
        void setBit(volatile uint32_t &reg, uint32_t bit);
        ZP_Error transmit(uint8_t* pData, uint16_t size);
        ZP_Error transmitDMA(uint8_t* pData, uint16_t size);
        ZP_Error receive(uint8_t *pData, uint16_t size, uint32_t deadline);
        ZP_Error restartDMA(uint8_t *pData, uint16_t size);

    private:
        UART_HandleTypeDef *huart;
        ZP_Error getState();
        ZP_Error errorToZpError();
        ZP_Error statusToZpError(HAL_StatusTypeDef status);
};
