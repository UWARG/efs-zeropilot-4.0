#pragma once
#include <stdint.h>
#include "zp_error.h"
#include "main.h"

typedef struct {
    GPIO_TypeDef* csGPIOPort;
    uint16_t csGPIOPin;
    SPI_HandleTypeDef* hspi;
} SpiCommonDevice_t;

// Functions to pull cs high and low
void spiCsLow(const SpiCommonDevice_t& device);
void spiCsHigh(const SpiCommonDevice_t& device);

// function to transmit 
ZP_Error spiTransmitData(const SpiCommonDevice_t& device, const uint8_t* txBuf, uint16_t size, uint32_t timeout);

// Function to receive data
ZP_Error spiTransmitReceiveData(const SpiCommonDevice_t& device, const uint8_t* txBuf, uint8_t* rxBuf, uint16_t size, uint32_t timeout);

// Function for SPI transmit and receive
ZP_Error spiTransmitReceiveDataDMA(const SpiCommonDevice_t& device, uint8_t* txBuf, uint8_t* rxBuf, uint16_t size);