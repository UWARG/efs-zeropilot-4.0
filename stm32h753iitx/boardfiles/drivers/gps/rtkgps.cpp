#include "rtkgps.hpp"
#include "stm32h7xx_hal.h"

HAL_StatusTypeDef RTKGPS::sendCorrectionData(const uint8_t *data, uint16_t len) {
    if (HAL_UART_Transmit(getHUART(), data, len, HAL_MAX_DELAY) == HAL_OK) return HAL_OK;
    else return HAL_ERROR;
};