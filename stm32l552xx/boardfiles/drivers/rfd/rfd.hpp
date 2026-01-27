#pragma once

#include "rfd_iface.hpp"
#include "rfd_defines.hpp"
#include "stm32l5xx_hal.h"
#include "error.h"

class RFD : public IRFD {

public:
    static RFD* instance; // assumes only one instance defined at a time

    RFD(UART_HandleTypeDef* huart);
    ~RFD();

    ZP_ERROR_e startReceive() override;
    ZP_ERROR_e transmit(const uint8_t* data, uint16_t size) override;
    ZP_ERROR_e receive(uint16_t *received_size, uint8_t* buffer, uint16_t bufferSize) override;

    // Getters
    UART_HandleTypeDef* getHuart() const;

    // DMA callback
    void receiveCallback(uint16_t size);

    // Start DMA
    void startReceive();

private:
    UART_HandleTypeDef* huart;
    uint8_t rxBuffer[BUFFER_SIZE];
    uint16_t readIndex;
    uint16_t writeIndex;
};
