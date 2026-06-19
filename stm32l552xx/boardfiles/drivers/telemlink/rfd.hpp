#pragma once

#include "telemlink_iface.hpp"
#include "rfd_defines.hpp"
#include "stm32l5xx_hal.h"
#include "zp_error.h"

class RFD : public ITelemLink {

public:
    static RFD* instance; // assumes only one instance defined at a time

    RFD(UART_HandleTypeDef* huart);
    ~RFD();

    ZP_ERROR_e transmit(const uint8_t* data, uint16_t size) override;
    ZP_ERROR_e receive(uint8_t* buffer, uint16_t bufferSize, uint16_t &received_size) override;

    // Getters
    UART_HandleTypeDef* getHUART() const;

    // DMA callback
    ZP_ERROR_e receiveCallback(uint16_t size);

    // Start DMA
    ZP_ERROR_e init();

private:
    ZP_ERROR_e getRXTransferSize(uint16_t idx, uint16_t& output);
    UART_HandleTypeDef* huart;
    uint8_t rxBuffer[BUFFER_SIZE];

    uint16_t readIndex = 0;
    uint16_t writeIndex = 0;

    uint16_t currentSize = 0;
    uint16_t lastIdx = 0;
};
