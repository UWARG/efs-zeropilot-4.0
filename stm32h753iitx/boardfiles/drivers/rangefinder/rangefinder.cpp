#include "rangefinder.hpp"

static constexpr uint8_t RANGEFINDER_I2C_ADDR = 0x10 << 1; // 7-bit address for the TF02-Pro rangefinder, shifted left by 1 for the HAL functions

static constexpr uint8_t FIRMWARE_VERSION_CMD[] = {0x5A, 0x04, 0x01, 0x5F};
static constexpr uint8_t OUTPUT_FORMAT_CM_CMD[] = {0x5A, 0x05, 0x05, 0x01, 0x65};
static constexpr uint8_t SAVE_CONFIG_CMD[] = {0x5A, 0x04, 0x11, 0x6F};
static constexpr uint8_t I2C_READ_CMD[] = {0x5A, 0x05, 0x00, 0x01, 0x60};

// Expected responses from commands
static constexpr uint8_t FIRMWARE_VERSION_RESPONSE[] = {0x5A, 0x07, 0x01}; // First three bytes of the firmware version response, the last two bytes are the actual version number
static constexpr uint8_t OUTPUT_FORMAT_CM_SUCCESS_RESPONSE[] = {0x5A, 0x05, 0x05, 0x01, 0x65};
static constexpr uint8_t SAVE_CONFIG_SUCCESS_RESPONSE[] = {0x5A, 0x05, 0x11, 0x00, 0x70};

static constexpr uint8_t READ_CMD_LEN = sizeof(I2C_READ_CMD);

static constexpr uint8_t DATA_FRAME_HEADER = 0x59;

static constexpr uint16_t STRGENTH_SATURATED = 65535;
static constexpr uint16_t DIST_SATURATED = 65534;
static constexpr uint16_t DIST_WEAK_SIGNAL = 4500;

Rangefinder::Rangefinder(I2C_HandleTypeDef *hi2c) : hi2c(hi2c) {}

int Rangefinder::init() {
    uint8_t recieveBuffer[6] = {0}; // Size 6 is the max response size for the rangefinder commands
    
    // Check firmware version to see if ithe rangefinder is present and alive
    if (HAL_I2C_Master_Transmit(hi2c, RANGEFINDER_I2C_ADDR, (uint8_t*)FIRMWARE_VERSION_CMD, sizeof(FIRMWARE_VERSION_CMD), HAL_MAX_DELAY) != HAL_OK) {
        return -1;
    }
    HAL_Delay(100); // Wait for the rangefinder to process the command, 100ms as suggested in the datasheet
    if (HAL_I2C_Master_Receive(hi2c, RANGEFINDER_I2C_ADDR, recieveBuffer, sizeof(FIRMWARE_VERSION_RESPONSE), HAL_MAX_DELAY) != HAL_OK) {
        return -1;
    }
    for (int i = 0; i < sizeof(FIRMWARE_VERSION_RESPONSE); i++) {
        if (recieveBuffer[i] != FIRMWARE_VERSION_RESPONSE[i]) {
            return -1; // Firmware version response does not match expected response
        }
    }

    // Configure output format to centimeters
    if (HAL_I2C_Master_Transmit(hi2c, RANGEFINDER_I2C_ADDR, (uint8_t*)OUTPUT_FORMAT_CM_CMD, sizeof(OUTPUT_FORMAT_CM_CMD), HAL_MAX_DELAY) != HAL_OK) {
        return -1;
    }
    HAL_Delay(100);
    if (HAL_I2C_Master_Receive(hi2c, RANGEFINDER_I2C_ADDR, recieveBuffer, sizeof(OUTPUT_FORMAT_CM_SUCCESS_RESPONSE), HAL_MAX_DELAY) != HAL_OK) {
        return -1;
    }
    for (int i = 0; i < sizeof(OUTPUT_FORMAT_CM_SUCCESS_RESPONSE); i++) {
        if (recieveBuffer[i] != OUTPUT_FORMAT_CM_SUCCESS_RESPONSE[i]) {
            return -1;
        }
    }

    // Maybe configrue the frame rate, but the default is 100Hz which is fine for now

    // Save configs
    if (HAL_I2C_Master_Transmit(hi2c, RANGEFINDER_I2C_ADDR, (uint8_t*)SAVE_CONFIG_CMD, sizeof(SAVE_CONFIG_CMD), HAL_MAX_DELAY) != HAL_OK) {
        return -1;
    }
    HAL_Delay(100);
    if (HAL_I2C_Master_Receive(hi2c, RANGEFINDER_I2C_ADDR, recieveBuffer, sizeof(SAVE_CONFIG_SUCCESS_RESPONSE), HAL_MAX_DELAY) != HAL_OK) {
        return -1;
    }
    for (int i = 0; i < sizeof(SAVE_CONFIG_SUCCESS_RESPONSE); i++) {
        if (recieveBuffer[i] != SAVE_CONFIG_SUCCESS_RESPONSE[i]) {
            return -1;
        }
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
            Checksum doesnt match, the data is corrupted
            When encountering a measured object with high reflectivity, strength = 65535 and the distance value will become 65534
            When the signal strength is insufficient and lower than 60, the distance value will become the maximum value of 4500
        */
        data.isValid = (computeChecksum() == rxBuffer[8]) && (rawStrength != STRGENTH_SATURATED)
                        && (rawDistance != DIST_SATURATED) && (rawDistance != DIST_WEAK_SIGNAL);
        data.isNew = true;
        data.distance = (float)rawDistance / 100.0f;
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
    HAL_I2C_Master_Receive_IT(hi2c, RANGEFINDER_I2C_ADDR, rxBuffer, READ_RESPONSE_LENGTH);
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
    HAL_I2C_Master_Transmit_IT(hi2c, RANGEFINDER_I2C_ADDR, (uint8_t*)I2C_READ_CMD, READ_CMD_LEN);
}

uint8_t Rangefinder::computeChecksum() {
    // Checksum is the lower 8 bits of the cumulative sum of number of first 8 bytes
    uint16_t sum = 0;
    for (uint8_t i = 0; i < 8; i++) {
        sum += rxBuffer[i];
    }
    return sum & 0xFF;
}



