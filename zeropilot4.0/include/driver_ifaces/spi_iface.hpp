#pragma once
#include <stdint.h>
#include "zp_error.h"
#include "main.h"

class SpiDevice {
    public:
        SpiDevice(GPIO_TypeDef *csGPIOPort, uint16_t csGPIOPin, SPI_HandleTypeDef *hspi);
        
        // Functions to pull cs high and low
        void spiCsLow();
        void spiCsHigh();

        // function to transmit 
        ZP_Error spiTransmitData(const uint8_t* txBuf, uint16_t size, uint32_t timeout);

        // Function to receive data
        ZP_Error spiTransmitReceiveData(const uint8_t* txBuf, uint8_t* rxBuf, uint16_t size, uint32_t timeout);

        // Function for SPI transmit and receive
        ZP_Error spiTransmitReceiveDataDMA(uint8_t* txBuf, uint8_t* rxBuf, uint16_t size);
    private: 
        GPIO_TypeDef* csGPIOPort;
        uint16_t csGPIOPin;
        SPI_HandleTypeDef* hspi;
};