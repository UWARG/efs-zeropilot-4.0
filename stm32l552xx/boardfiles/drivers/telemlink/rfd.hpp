#pragma once

#include "telemlink_iface.hpp"
#include "rfd_defines.hpp"
#include "stm32l5xx_hal.h"

class RFD : public ITelemLink {

public:
    static RFD* instance; // assumes only one instance defined at a time

    RFD(UART_HandleTypeDef* huart);
    ~RFD();

    void transmit(const uint8_t* data, uint16_t size) override;
    uint16_t receive(uint8_t* buffer, uint16_t bufferSize) override;

    // Getters
    UART_HandleTypeDef* getHUART() const;

    // DMA callback
    void receiveCallback(uint16_t size);

    // Start DMA
    void init();

private:
    uint16_t getRXTransferSize(uint16_t idx);
    UART_HandleTypeDef* huart;
    uint8_t rxBuffer[BUFFER_SIZE];

    uint16_t readIndex = 0;
    uint16_t writeIndex = 0;

    uint16_t currentSize = 0;
    uint16_t lastIdx = 0;
};
