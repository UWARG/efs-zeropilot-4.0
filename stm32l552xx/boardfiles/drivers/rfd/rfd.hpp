#pragma once

#include "rfd_iface.hpp"
#include "rfd_defines.hpp"
#include "stm32l5xx_hal.h"

class RFD : public IRFD {

public:
    static RFD* instance; // assumes only one instance defined at a time

    RFD(UART_HandleTypeDef* huart);
    ~RFD();

    void transmit(const uint8_t* data, uint16_t size) override;
    uint16_t receive(uint8_t* buffer, uint16_t bufferSize) override;

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
