#include "tf02pro.hpp"

static constexpr uint8_t TF02PRO_I2C_ADDR = 0x10 << 1; // 7-bit address for the TF02-Pro, shifted left by 1 for the HAL functions

static constexpr uint8_t FIRMWARE_VERSION_CMD[] = {0x5A, 0x04, 0x01, 0x5F};
static constexpr uint8_t OUTPUT_FORMAT_CM_CMD[] = {0x5A, 0x05, 0x05, 0x01, 0x65};
static constexpr uint8_t SAVE_CONFIG_CMD[] = {0x5A, 0x04, 0x11, 0x6F};
static constexpr uint8_t I2C_READ_CMD[] = {0x5A, 0x05, 0x00, 0x01, 0x60};

// Expected responses from commands
static constexpr uint8_t FIRMWARE_VERSION_RESPONSE[] = {0x5A, 0x07, 0x01}; // First three bytes of the firmware version response, the last two bytes are the actual version number
static constexpr uint8_t OUTPUT_FORMAT_CM_SUCCESS_RESPONSE[] = {0x5A, 0x05, 0x05, 0x01, 0x65};
static constexpr uint8_t SAVE_CONFIG_SUCCESS_RESPONSE[] = {0x5A, 0x05, 0x11, 0x00, 0x70};

static constexpr uint8_t MAX_CMD_RESPONSE_LENGTH = 6;

static constexpr uint8_t READ_CMD_LEN = sizeof(I2C_READ_CMD);

static constexpr uint8_t DATA_FRAME_HEADER = 0x59;

static constexpr uint16_t STRENGTH_SATURATED = 65535;
static constexpr uint16_t DIST_SATURATED = 65534;
static constexpr uint16_t DIST_WEAK_SIGNAL = 4500;

static constexpr uint32_t TF02PRO_PROCESS_CMD_DELAY_MS = 100; // Wait time for TF02-Pro to process the command, as suggested in the datasheet

Rangefinder::Rangefinder(I2C_HandleTypeDef *hi2c) : hi2c(hi2c) {}

int Rangefinder::init() {
    // Check firmware version to see if the rangefinder is present and alive
    if (sendCmdCheckResp(FIRMWARE_VERSION_CMD, sizeof(FIRMWARE_VERSION_CMD), 
                        FIRMWARE_VERSION_RESPONSE, sizeof(FIRMWARE_VERSION_RESPONSE)) != HAL_OK) {
        return -1;
    }

    // Configure output format to centimeters
    if (sendCmdCheckResp(OUTPUT_FORMAT_CM_CMD, sizeof(OUTPUT_FORMAT_CM_CMD), 
                        OUTPUT_FORMAT_CM_SUCCESS_RESPONSE, sizeof(OUTPUT_FORMAT_CM_SUCCESS_RESPONSE)) != HAL_OK) {
        return -1;
    }

    // Maybe configure the frame rate, but the default is 100Hz which is fine for now

    // Save configs
    if (sendCmdCheckResp(SAVE_CONFIG_CMD, sizeof(SAVE_CONFIG_CMD), 
                        SAVE_CONFIG_SUCCESS_RESPONSE, sizeof(SAVE_CONFIG_SUCCESS_RESPONSE)) != HAL_OK) {
        return -1;
    }
    return 0;
}

RangefinderData_t Rangefinder::readData() {
    // No frame ready yet: kick off the I2C transfer for first call or restart it if the previous one never completed
    if (!dataFilled) {
        restartTransfer();
        // Report the sample as neither valid nor new
        data.isValid = false;
        data.isNew = false;
        return data;
    }
    dataFilled = false;
    
    // Parse the received data
    if (rxBuffer[0] == DATA_FRAME_HEADER && rxBuffer[1] == DATA_FRAME_HEADER) { // Check the frame header so we don't parse a corrupted frame
        uint16_t rawDistance = rxBuffer[3] << 8 | rxBuffer[2];
        uint16_t rawStrength = rxBuffer[5] << 8 | rxBuffer[4];
        int16_t rawTemp = (rxBuffer[7] << 8 | rxBuffer[6]) / 8 - 256;

        /* 
        Check if the received frame is valid: 
            Checksum doesn't match, the data is corrupted
            When encountering a measured object with high reflectivity, strength = 65535 and the distance value will become 65534
            When the signal strength is insufficient and lower than 60, the distance value will become the maximum value of 4500
        */
        data.isValid = (computeChecksum() == rxBuffer[8]) && (rawStrength != STRENGTH_SATURATED)
                        && (rawDistance != DIST_SATURATED) && (rawDistance != DIST_WEAK_SIGNAL);
        data.isNew = true;
        data.distance = (float)rawDistance / 100.0f; // Convert cm to m
        data.signalStrength = rawStrength;
        data.temp = rawTemp;
    } else {
        data.isValid = false; // The frame is corrupted, make the data not valid
    }

    // Kick off the next transfer so a fresh frame is ready by the next readData() call
    restartTransfer();

    return data;
}

void Rangefinder::txCallback() {
    HAL_I2C_Master_Receive_IT(hi2c, TF02PRO_I2C_ADDR, rxBuffer, READ_RESPONSE_LENGTH);
}

void Rangefinder::rxCallback() {
    dataFilled = true;
}

void Rangefinder::errorCallback() {
    restartTransfer();
}

I2C_HandleTypeDef *Rangefinder::getI2C() {
    return hi2c;
}

void Rangefinder::restartTransfer() {
    // AM calls every loop(1kHz) but rangefinder frame rate is 100Hz, so limit the transfer to the frame rate
    if (HAL_GetTick() - lastTransferTick >= 10) {
        HAL_I2C_Master_Transmit_IT(hi2c, TF02PRO_I2C_ADDR, (uint8_t*)I2C_READ_CMD, READ_CMD_LEN);
        lastTransferTick = HAL_GetTick();
    }
}

uint8_t Rangefinder::computeChecksum() {
    // Checksum is the lower 8 bits of the cumulative sum of number of first 8 bytes
    uint16_t sum = 0;
    for (uint8_t i = 0; i < 8; i++) {
        sum += rxBuffer[i];
    }
    return sum & 0xFF;
}

HAL_StatusTypeDef Rangefinder::writeDataBlocking(uint8_t* cmd, uint16_t size, uint32_t delay) {
    HAL_StatusTypeDef status = HAL_I2C_Master_Transmit(hi2c, TF02PRO_I2C_ADDR, cmd, size, delay);
    if (status == HAL_ERROR) {
        return HAL_ERROR; // Change to ZP Error Standards later
    } else if (status == HAL_BUSY) {
        return HAL_BUSY; // Change to ZP Error Standards later
    } else if (status == HAL_TIMEOUT) {
        return HAL_TIMEOUT; // Change to ZP Error Standards later
    }
    return HAL_OK;
}

HAL_StatusTypeDef Rangefinder::readDataBlocking(uint8_t* receiveBuffer, uint16_t size, uint32_t delay) {
    HAL_StatusTypeDef status = HAL_I2C_Master_Receive(hi2c, TF02PRO_I2C_ADDR, receiveBuffer, size, delay);
    if (status == HAL_ERROR) {
        return HAL_ERROR; // Change to ZP Error Standards later
    } else if (status == HAL_BUSY) {
        return HAL_BUSY; // Change to ZP Error Standards later
    } else if (status == HAL_TIMEOUT) {
        return HAL_TIMEOUT; // Change to ZP Error Standards later
    }
    return HAL_OK;
}

HAL_StatusTypeDef Rangefinder::sendCmdCheckResp(const uint8_t *cmd, uint16_t cmdSize,
                                                const uint8_t *expectedResp, uint16_t expectedRespSize) {
    uint8_t receiveBuffer[MAX_CMD_RESPONSE_LENGTH] = {0};

    HAL_StatusTypeDef status = writeDataBlocking((uint8_t*)cmd, cmdSize, HAL_MAX_DELAY);
    if (status != HAL_OK) {
        return status;
    }

    HAL_Delay(TF02PRO_PROCESS_CMD_DELAY_MS);

    status = readDataBlocking(receiveBuffer, expectedRespSize, HAL_MAX_DELAY);
    if (status != HAL_OK) {
        return status;
    }

    for (uint8_t i = 0; i < expectedRespSize; i++) {
        if (receiveBuffer[i] != expectedResp[i]) {
            return HAL_ERROR; // Device responded but does not match the expected response
        }
    }
    return HAL_OK;
}
