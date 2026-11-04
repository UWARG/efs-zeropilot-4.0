#pragma once
#include <stdint.h>
#include "zp_error.h"
#include "main.h"

inline ZP_Error halToZpError(HAL_StatusTypeDef status, uint32_t errorCode) {
    typedef struct {
        uint32_t halErrorBit;
        ZP_Error zpError;
    } errorMap;

    static constexpr errorMap halErrorMapTable[] = {
        {HAL_SPI_ERROR_FLAG, ZP_ERROR_EXT_API | ZP_ERROR_TIMEOUT},
        {HAL_SPI_ERROR_DMA, ZP_ERROR_EXT_API | ZP_ERROR_BUSY}
    };

    static constexpr ZP_Error statusMap[] = {
        ZP_ERROR_OK, 
        ZP_ERROR_EXT_API | ZP_ERROR_FAIL,
        ZP_ERROR_EXT_API | ZP_ERROR_BUSY,
        ZP_ERROR_EXT_API | ZP_ERROR_TIMEOUT
    };

    if (status == HAL_ERROR) {
        for (const errorMap& element : halErrorMapTable) {
            if (element.halErrorBit & errorCode) { return element.zpError; }
        }
    }

    if (status > HAL_TIMEOUT) { return ZP_ERROR_EXT_API | ZP_ERROR_FAIL; }
    return statusMap[status];
}
class ISpi {
    public:
        ISpi(GPIO_TypeDef *csGPIOPort, uint16_t csGPIOPin, SPI_HandleTypeDef *hspi);
        
        void csLow();
        void csHigh();

        ZP_Error transmit(uint8_t* txBuf, uint16_t size, uint32_t timeout);
        ZP_Error transceive(uint8_t* txBuf, uint8_t* rxBuf, uint16_t size, uint32_t timeout);
        ZP_Error transceiveDMA(uint8_t* txBuf, uint8_t* rxBuf, uint16_t size);

    private: 
        GPIO_TypeDef* csGPIOPort;
        uint16_t csGPIOPin;
        SPI_HandleTypeDef* hspi;
};