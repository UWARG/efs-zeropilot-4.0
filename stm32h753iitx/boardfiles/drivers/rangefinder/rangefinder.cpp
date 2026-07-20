#include "rangefinder.hpp"

static constexpr uint8_t RANGEFINDER_I2C_ADDR = 0x10 << 1; // 7-bit address for the TF02-Pro rangefinder, shifted left by 1 for the HAL functions

static constexpr uint8_t FIRMWARE_VERSION_CMD[] = {0x5A, 0x04, 0x01, 0x5F};
static constexpr uint8_t RESET_CMD[] = {0x5A, 0x04, 0x02, 0x60};
static constexpr uint8_t OUTPUT_FORMAT_CM_CMD[] = {0x5A, 0x05, 0x05, 0x01, 0x65};
static constexpr uint8_t SAVE_CONFIG_CMD[] = {0x5A, 0x04, 0x11, 0x6F};
static constexpr uint8_t I2C_READ_CMD[] = {0x5A, 0x05, 0x00, 0x01, 0x60};

// Expected responses from commands
static constexpr uint8_t FIRMWARE_VERSION_RESPONSE[] = {0x5A, 0x05, 0x01}; // First three bytes of the firmware version response, the last two bytes are the actual version number
static constexpr uint8_t RESET_SUCCESS_RESPONSE[] = {0x5A, 0x05, 0x02, 0x00, 0x61};
static constexpr uint8_t RESET_FAILURE_RESPONSE[] = {0x5A, 0x05, 0x02, 0x01, 0x62};
static constexpr uint8_t OUTPUT_FORMAT_CM_SUCCESS_RESPONSE[] = {0x5A, 0x05, 0x05, 0x01, 0x65};
static constexpr uint8_t SAVE_CONFIG_SUCCESS_RESPONSE[] = {0x5A, 0x05, 0x11, 0x00, 0x70};

static constexpr uint8_t READ_CMD_LEN = sizeof(I2C_READ_CMD);

static constexpr uint8_t DATA_FRAME_HEADER = 0x59;

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

    // Perform a system reset 
    if (HAL_I2C_Master_Transmit(hi2c, RANGEFINDER_I2C_ADDR, (uint8_t*)RESET_CMD, sizeof(RESET_CMD), HAL_MAX_DELAY) != HAL_OK) {
        return -1;
    }
    HAL_Delay(100); 
    if (HAL_I2C_Master_Receive(hi2c, RANGEFINDER_I2C_ADDR, recieveBuffer, sizeof(RESET_SUCCESS_RESPONSE), HAL_MAX_DELAY) != HAL_OK) {
        return -1;
    }
    for (int i = 0; i < sizeof(RESET_SUCCESS_RESPONSE); i++) {
        if (recieveBuffer[i] != RESET_SUCCESS_RESPONSE[i]) {
            return -1; 
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
    return true;
}

RangefinderData_t Rangefinder::readData() {
    // Kick off DMA transfer for the first transfer, the first readData() call will return empty data
    if (!dataFilled) {
        restartDma();
        return data;
    }
    dataFilled = false;
    
    // Parse the recieved data
    if (rxBuffer[0] == DATA_FRAME_HEADER && rxBuffer[1] == DATA_FRAME_HEADER) { // Check the frame header so we dont parse a corrupted frame
        data.distance = rxBuffer[3] << 8 | rxBuffer[2];
        data.signalStrength = rxBuffer[5] << 8 | rxBuffer[4];
        data.temp = (rxBuffer[7] << 8 | rxBuffer[6]) / 8 - 256;

        if (computeChecksum() != rxBuffer[8]) { // The checksum doesnt match, the data is corrupted
            data.isValid = false;
        }
        data.isValid = true;
    } 

    // Kick off another DMA transfer we have new when next readData() is called
    restartDma();

    return data;
}

void Rangefinder::txCallback() {
    HAL_I2C_Master_Receive_DMA(hi2c, RANGEFINDER_I2C_ADDR, rxBuffer, READ_RESPONSE_LENGTH);
}

void Rangefinder::rxCallback() {
    dataFilled = true;
}

void Rangefinder::errorCallback() {
    while(1);
}

I2C_HandleTypeDef *Rangefinder::getI2C() {
    return hi2c;
}

void Rangefinder::restartDma() {
    HAL_I2C_Master_Transmit_DMA(hi2c, RANGEFINDER_I2C_ADDR, (uint8_t*)I2C_READ_CMD, READ_CMD_LEN);
}

uint8_t Rangefinder::computeChecksum() {
    // Checksum is the lower 8 bits of the cumulative sum of number of first 8 bytes
    uint16_t sum = 0;
    for (uint8_t i = 0; i < 8; i++) {
        sum += rxBuffer[i];
    }
    return sum & 0xFF;
}



