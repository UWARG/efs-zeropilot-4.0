#include "uart_iface.hpp"

UARTInterface::UARTInterface(UART_HandleTypeDef *huart) {
    this->huart = huart;
}

UART_HandleTypeDef* UARTInterface::getHuart(){
    return this->huart;
}

void UARTInterface::setBit(uint32_t reg, uint32_t bit){
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
    // either OK or FAIL
}

ZP_Error UARTInterface::transmitDMA(uint8_t* pData, uint16_t size){
    HAL_StatusTypeDef status = HAL_UARTEx_ReceiveToIdle_DMA(huart, pData, size);
    return statusToZpError(status);
    // either BUSY or FAIL or OK
}

ZP_Error UARTInterface::receive(uint8_t *pData, uint16_t size, uint32_t deadline){
    int32_t remainingTick = (int32_t)(deadline - HAL_GetTick());
    if (remainingTick <= 0) return ZP_ERROR_TIMEOUT;

    HAL_StatusTypeDef status = HAL_UART_Receive(huart, pData, size, (uint32_t)remainingTick);
    return statusToZpError(status);
    // TIMEOUT, OK or FAIL
}

ZP_Error UARTInterface::restartDMA(uint8_t *pData, uint16_t size){
    HAL_StatusTypeDef status = HAL_UARTEx_ReceiveToIdle_DMA(huart, pData, size);
    // line below only runs for GPS, confirm if we care about this for rc drivers...
    __HAL_DMA_DISABLE_IT(huart->hdmarx, DMA_IT_HT); 
    return statusToZpError(status);
    // gps: OK or FAIL
    // rc: BUSY , FAIL or OK
}
