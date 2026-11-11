#pragma once

#include "stm32l5xx_hal.h"
#include "rangefinder_iface.hpp"

class Rangefinder : public IRangefinder {
public:
    Rangefinder(I2C_HandleTypeDef *hi2c);

    ZP_Error init() override;
    ZP_Error readData(RangefinderData_t &outData) override;

    void txCallback();
    void rxCallback();
    void errorCallback();

    I2C_HandleTypeDef *getI2C();

private:
    I2C_HandleTypeDef *hi2c;
    RangefinderData_t data = {};
    volatile bool dataFilled = false;

    static constexpr uint8_t READ_RESPONSE_LENGTH = 9;
    uint8_t rxBuffer[READ_RESPONSE_LENGTH] = {0};

    ZP_Error restartTransfer();
    uint8_t computeChecksum();
    uint32_t lastTransferTick = 0;

    ZP_Error writeDataBlocking(uint8_t* cmd, uint16_t cmdSize, uint32_t delay);
    ZP_Error readDataBlocking(uint8_t* receiveBuffer, uint16_t size, uint32_t delay);
    ZP_Error sendCmdCheckResp(const uint8_t *cmd, uint16_t cmdSize, const uint8_t *expectedResp, uint16_t expectedRespSize);
};
