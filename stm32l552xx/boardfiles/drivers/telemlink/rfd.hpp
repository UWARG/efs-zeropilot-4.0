#pragma once

#include "telemlink_iface.hpp"
#include "rfd_defines.hpp"
#include "zp_error.h"
#include "stm32l5xx_hal.h"

class RFD : public ITelemLink {

public:
    static RFD* instance; // assumes only one instance defined at a time

    RFD(UART_HandleTypeDef* huart);
    ~RFD();

    ZP_Error transmit(const uint8_t* data, uint16_t size) override;
    ZP_Error receive(uint8_t* buffer, uint16_t bufferSize, uint16_t &received_size) override;

    // Getters
    UART_HandleTypeDef* getHuart() const;

    // DMA callback
    ZP_Error receiveCallback(uint16_t size);

    // Start DMA
    ZP_Error init();

    // Restart reception after a UART error aborted the DMA
    ZP_Error restartRx();

private:
    ZP_Error getRXTransferSize(uint16_t idx, uint16_t& output);
    UART_HandleTypeDef* huart;
    uint8_t rxBuffer[BUFFER_SIZE];
    uint8_t txBuffer[TX_BUFFER_SIZE]; // Owned by the driver so a caller cannot overwrite a transfer in flight

    uint16_t readIndex = 0;
    uint16_t writeIndex = 0;

    uint16_t currentSize = 0;
    uint16_t lastIdx = 0;
};
