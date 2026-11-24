#pragma once
#include <stdint.h>
#include "zp_error.h"
#include "main.h"

inline ZP_Error halToZpError(HAL_StatusTypeDef status, uint32_t errorCode) {
    typedef struct {
        uint32_t halErrorBit;
        ZP_Error zpError;
    } SpiErrorMap_t;

    static constexpr SpiErrorMap_t SPI_ERROR_MAP[] = {
        {HAL_SPI_ERROR_MODF,  ZP_ERROR_SPI_MODF},
        {HAL_SPI_ERROR_CRC,   ZP_ERROR_SPI_CRC},
        {HAL_SPI_ERROR_OVR,   ZP_ERROR_SPI_OVR},
        {HAL_SPI_ERROR_FRE,   ZP_ERROR_SPI_FRE},
        {HAL_SPI_ERROR_DMA,   ZP_ERROR_SPI_DMA},
        {HAL_SPI_ERROR_FLAG,  ZP_ERROR_SPI_FLAG},
        {HAL_SPI_ERROR_ABORT, ZP_ERROR_SPI_ABORT}
    };

    if (status == HAL_OK) return ZP_ERROR_OK;
    if (status == HAL_BUSY) return ZP_ERROR_EXT_API | ZP_ERROR_BUSY;
    if (status == HAL_TIMEOUT) return ZP_ERROR_EXT_API | ZP_ERROR_TIMEOUT;

    ZP_Error result = ZP_ERROR_EXT_API;
    for (const SpiErrorMap_t& element : SPI_ERROR_MAP) {
        if (errorCode & element.halErrorBit) result |= element.zpError;
    }

    if (result == ZP_ERROR_EXT_API) result |= ZP_ERROR_FAIL;
    return result;
}

class ISpi {
    public:
        ISpi(GPIO_TypeDef *csGPIOPort, uint16_t csGPIOPin, SPI_HandleTypeDef *hspi);
        
        ZP_Error csLow();
        ZP_Error csHigh();

        ZP_Error transmit(uint8_t* txBuf, uint16_t size, uint32_t timeout);
        ZP_Error transceive(uint8_t* txBuf, uint8_t* rxBuf, uint16_t size, uint32_t timeout);
        ZP_Error transceiveDMA(uint8_t* txBuf, uint8_t* rxBuf, uint16_t size);

    private: 
        GPIO_TypeDef* csGPIOPort;
        uint16_t csGPIOPin;
        SPI_HandleTypeDef* hspi;
};