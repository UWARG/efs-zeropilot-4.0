#include "uart_iface.hpp"

UARTInterface::UARTInterface(UART_HandleTypeDef *huart) {
    this->huart = huart;
}

ZP_Error UARTInterface::errorToZpError() {
    switch (HAL_UART_GetError(this->huart)) {
        case HAL_UART_ERROR_NONE:   return ZP_ERROR_OK;
        case HAL_UART_ERROR_PE:     return ZP_ERROR_CRC;
        case HAL_UART_ERROR_NE:     return ZP_ERROR_INVALID_ARG;
        case HAL_UART_ERROR_FE:     return ZP_ERROR_INVALID_ARG;
        case HAL_UART_ERROR_ORE:    return ZP_ERROR_MEMORY_OVERFLOW;
        case HAL_UART_ERROR_RTO:    return ZP_ERROR_TIMEOUT;
        case HAL_UART_ERROR_DMA:    return ZP_ERROR_FAIL;
        default : return ZP_ERROR_FAIL;
    }
}

ZP_Error UARTInterface::statusToZpError(HAL_StatusTypeDef status) {
    switch (status) {
        case HAL_TIMEOUT:       return ZP_ERROR_EXT_API | ZP_ERROR_TIMEOUT;
        case HAL_BUSY:          return ZP_ERROR_EXT_API | ZP_ERROR_BUSY;
        case HAL_OK:            return ZP_ERROR_OK;
        case HAL_ERROR:         return errorToZpError();
        default:                return ZP_ERROR_EXT_API | ZP_ERROR_FAIL;
    }
}

UART_HandleTypeDef* UARTInterface::getHuart(){
    return this->huart;
}

void UARTInterface::setBit(volatile uint32_t &reg, uint32_t bit){
    SET_BIT(reg, bit);
}

ZP_Error UARTInterface::getState(){
    if (huart->gState != HAL_UART_STATE_READY) {
        return ZP_ERROR_EXT_API | ZP_ERROR_BUSY;
    }
    return ZP_ERROR_OK;
}

ZP_Error UARTInterface::transmit(uint8_t* pData, uint16_t size){
    HAL_StatusTypeDef status = HAL_UART_Transmit(huart, pData, size, HAL_MAX_DELAY);
    return statusToZpError(status);
}

ZP_Error UARTInterface::transmitDMA(uint8_t* pData, uint16_t size){
    HAL_StatusTypeDef status = HAL_UART_Transmit_DMA(huart, pData, size);
    return statusToZpError(status);
}

ZP_Error UARTInterface::receive(uint8_t *pData, uint16_t size, uint32_t deadline){
    int32_t remainingTick = (int32_t)(deadline - HAL_GetTick());
    if (remainingTick <= 0) return ZP_ERROR_TIMEOUT;

    HAL_StatusTypeDef status = HAL_UART_Receive(huart, pData, size, (uint32_t)remainingTick);
    return statusToZpError(status);
}

ZP_Error UARTInterface::restartDMA(uint8_t *pData, uint16_t size){
    HAL_StatusTypeDef status = HAL_UARTEx_ReceiveToIdle_DMA(huart, pData, size);
    // line below only runs for GPS, confirm if we care about this for rc drivers...
    __HAL_DMA_DISABLE_IT(huart->hdmarx, DMA_IT_HT);
    return statusToZpError(status);
}