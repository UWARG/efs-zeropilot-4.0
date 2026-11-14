#include "eeprom.hpp"
#include "eeprom_iface.hpp"

#include <cstring>
#include <cstdint>

namespace {
    // Instruction bytes (datasheet Table 3-1)
    constexpr uint8_t CMD_READ  = 0x03;
    constexpr uint8_t CMD_WRITE = 0x02;
    constexpr uint8_t CMD_WREN  = 0x06;
    constexpr uint8_t CMD_RDSR  = 0x05;

    // Status register: bit 0 is "write in progress". VERIFY in the datasheet.
    constexpr uint8_t STATUS_WIP_MASK = 0x01;

    // 25LC128: 16384 bytes, 64 byte pages. VERIFY page size in the datasheet.
    constexpr uint16_t PAGE_SIZE = 64;
    constexpr uint32_t MEMORY_SIZE = 16384;

    // Datasheet lists 5 ms as the max write cycle time, so 10 ms leaves margin.
    constexpr uint32_t WRITE_TIMEOUT_MS = 10;
    constexpr uint32_t SPI_TIMEOUT_MS = 100;

    constexpr uint8_t HEADER_SIZE = 3;  // 1 command byte + 2 address bytes
}

EEPROM::EEPROM(SPI_HandleTypeDef *spi, GPIO_TypeDef *csPort, uint16_t csPin) :
    spi(spi),
    csPort(csPort),
    csPin(csPin) {}

void EEPROM::select() {
    HAL_GPIO_WritePin(csPort, csPin, GPIO_PIN_RESET);
}

void EEPROM::deselect() {
    HAL_GPIO_WritePin(csPort, csPin, GPIO_PIN_SET);
}

int EEPROM::writeEnable() {
    const uint8_t cmd = CMD_WREN;

    select();
    HAL_StatusTypeDef status = HAL_SPI_Transmit(spi, &cmd, 1, SPI_TIMEOUT_MS);
    deselect();  // CS must go high here or the chip never latches write enable

    return (status == HAL_OK) ? EEPROM_OK : EEPROM_ERR_SPI;
}

int EEPROM::readStatus(uint8_t &status) {
    // SPI sends and receives at the same time, so the second byte is a dummy
    // that gives the chip a clock to answer on.
    const uint8_t tx[2] = {CMD_RDSR, 0x00};
    uint8_t rx[2] = {0, 0};

    select();
    HAL_StatusTypeDef res = HAL_SPI_TransmitReceive(spi, tx, rx, 2, SPI_TIMEOUT_MS);
    deselect();

    if (res != HAL_OK) {
        return EEPROM_ERR_SPI;
    }

    status = rx[1];
    return EEPROM_OK;
}

int EEPROM::waitUntilReady(uint32_t timeoutMs) {
    uint32_t start = HAL_GetTick();

    while (true) {
        uint8_t status = 0;
        int res = readStatus(status);
        if (res != EEPROM_OK) {
            return res;
        }

        if ((status & STATUS_WIP_MASK) == 0) {
            return EEPROM_OK;
        }

        if (HAL_GetTick() - start > timeoutMs) {
            return EEPROM_ERR_TIMEOUT;
        }
    }
}

int EEPROM::read(uint16_t address, uint8_t *buffer, uint16_t length) {
    if (length == 0) {
        return EEPROM_OK;
    }
    if (buffer == nullptr || (static_cast<uint32_t>(address) + length) > MEMORY_SIZE) {
        return EEPROM_ERR_ARG;
    }

    const uint8_t header[HEADER_SIZE] = {
        CMD_READ,
        static_cast<uint8_t>(address >> 8),
        static_cast<uint8_t>(address & 0xFF)
    };

    // One visit: command and address, then the chip streams the bytes back.
    select();
    HAL_StatusTypeDef res = HAL_SPI_Transmit(spi, header, HEADER_SIZE, SPI_TIMEOUT_MS);
    if (res == HAL_OK) {
        res = HAL_SPI_Receive(spi, buffer, length, SPI_TIMEOUT_MS);
    }
    deselect();

    return (res == HAL_OK) ? EEPROM_OK : EEPROM_ERR_SPI;
}

int EEPROM::write(uint16_t address, const uint8_t *data, uint16_t length) {
    if (length == 0) {
        return EEPROM_OK;
    }
    if (data == nullptr || (static_cast<uint32_t>(address) + length) > MEMORY_SIZE) {
        return EEPROM_ERR_ARG;
    }

    uint16_t written = 0;

    while (written < length) {
        uint16_t chunkAddress = address + written;

        // A single write can't cross the end of a page (the chip would wrap
        // around and overwrite the start of the page), so cut at the boundary.
        uint16_t roomInPage = PAGE_SIZE - (chunkAddress % PAGE_SIZE);
        uint16_t remaining = length - written;
        uint16_t chunkLength = (remaining < roomInPage) ? remaining : roomInPage;

        uint8_t txBuf[HEADER_SIZE + PAGE_SIZE];
        txBuf[0] = CMD_WRITE;
        txBuf[1] = static_cast<uint8_t>(chunkAddress >> 8);
        txBuf[2] = static_cast<uint8_t>(chunkAddress & 0xFF);
        std::memcpy(&txBuf[HEADER_SIZE], &data[written], chunkLength);

        int res = writeEnable();
        if (res != EEPROM_OK) {
            return res;
        }

        // One visit: CS stays low for the whole message. The chip starts
        // saving when CS goes high.
        select();
        HAL_StatusTypeDef status = HAL_SPI_Transmit(spi, txBuf, HEADER_SIZE + chunkLength, SPI_TIMEOUT_MS);
        deselect();

        if (status != HAL_OK) {
            return EEPROM_ERR_SPI;
        }

        res = waitUntilReady(WRITE_TIMEOUT_MS);
        if (res != EEPROM_OK) {
            return res;
        }

        written += chunkLength;
    }

    return EEPROM_OK;
}
