#include "eeprom.hpp"

#include <cstring>

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

// ---- SELFTEST BEGIN (temporary, delete before committing) ----
#ifdef EEPROM_SELFTEST

namespace {
    // Test area is high in memory, away from anything the real firmware will use.
    constexpr uint16_t TEST_BASE = 0x3F00;
    constexpr uint16_t LAST_ADDRESS = 0x3FFF;
    constexpr uint16_t COUNTER_ADDRESS = 0x3FF0;
    constexpr uint16_t MAX_LEN = 150;

    void fillPattern(uint8_t *buf, uint16_t length, uint8_t seed) {
        for (uint16_t i = 0; i < length; i++) {
            buf[i] = static_cast<uint8_t>(seed + i);
        }
    }

    bool matches(const uint8_t *a, const uint8_t *b, uint16_t length) {
        return std::memcmp(a, b, length) == 0;
    }

    bool readByte(EEPROM &eeprom, uint16_t address, uint8_t &value) {
        return eeprom.read(address, &value, 1) == EEPROM_OK;
    }

    // Writes a known pattern, reads it back, and checks it is identical.
    bool roundTrip(EEPROM &eeprom, uint16_t address, uint16_t length, uint8_t seed) {
        uint8_t out[MAX_LEN];
        uint8_t in[MAX_LEN];

        if (length > MAX_LEN) {
            return false;
        }

        fillPattern(out, length, seed);
        std::memset(in, 0, length);

        if (eeprom.write(address, out, length) != EEPROM_OK) {
            return false;
        }
        if (eeprom.read(address, in, length) != EEPROM_OK) {
            return false;
        }
        return matches(out, in, length);
    }
}

int EEPROM::selfTest() {
    // 1: a single byte survives a write and a read.
    if (!roundTrip(*this, TEST_BASE, 1, 0xA5)) {
        return 1;
    }

    // 2: several bytes inside one page.
    if (!roundTrip(*this, TEST_BASE + 0x10, 8, 0x20)) {
        return 2;
    }

    // 3: a write that crosses a page boundary (0x3F40 is the start of a page).
    //    The bytes just outside the written range must not change.
    uint8_t beforeLow = 0, beforeHigh = 0, afterLow = 0, afterHigh = 0;
    if (!readByte(*this, TEST_BASE + 0x3D, beforeLow) ||
        !readByte(*this, TEST_BASE + 0x43, beforeHigh)) {
        return 3;
    }
    if (!roundTrip(*this, TEST_BASE + 0x3E, 5, 0x30)) {
        return 3;
    }
    if (!readByte(*this, TEST_BASE + 0x3D, afterLow) ||
        !readByte(*this, TEST_BASE + 0x43, afterHigh)) {
        return 3;
    }
    if (afterLow != beforeLow || afterHigh != beforeHigh) {
        return 3;
    }

    // 4: no wrap-around. If the page split were wrong, the chip would have
    //    wrapped back to the start of the page and overwritten TEST_BASE,
    //    which test 1 set to 0xA5.
    uint8_t first = 0;
    if (!readByte(*this, TEST_BASE, first) || first != 0xA5) {
        return 4;
    }

    // 5: a long write that starts mid-page and spans four pages.
    if (!roundTrip(*this, TEST_BASE + 0x30, 150, 0x40)) {
        return 5;
    }

    // 6: zero-length calls are harmless and report success.
    uint8_t dummy = 0;
    if (read(TEST_BASE, &dummy, 0) != EEPROM_OK) {
        return 6;
    }
    if (write(TEST_BASE, &dummy, 0) != EEPROM_OK) {
        return 6;
    }

    // 7: bad arguments are rejected instead of touching the chip.
    uint8_t five[5] = {1, 2, 3, 4, 5};
    if (write(LAST_ADDRESS - 1, five, 5) != EEPROM_ERR_ARG) {
        return 7;  // runs past the end of memory
    }
    if (read(LAST_ADDRESS - 1, five, 5) != EEPROM_ERR_ARG) {
        return 7;
    }
    if (write(TEST_BASE, nullptr, 1) != EEPROM_ERR_ARG) {
        return 7;
    }
    if (read(TEST_BASE, nullptr, 1) != EEPROM_ERR_ARG) {
        return 7;
    }

    // 8: the very last address works.
    if (!roundTrip(*this, LAST_ADDRESS, 1, 0x77)) {
        return 8;
    }

    return 0;
}

// Proves the memory is non-volatile. Run this once per power-up and watch the
// number. A brand new chip is all 0xFF, so the first run gives 0, then 1, 2, 3
// after each power cycle. If it resets to 0 every time, nothing is being saved.
int EEPROM::bootCounter() {
    uint8_t count = 0;

    if (read(COUNTER_ADDRESS, &count, 1) != EEPROM_OK) {
        return -1;
    }

    count = static_cast<uint8_t>(count + 1);

    if (write(COUNTER_ADDRESS, &count, 1) != EEPROM_OK) {
        return -2;
    }

    return count;
}

// Tests the private helpers directly. Result: 0 means pass, 101 and up is the
// first failing check.
int EEPROM::selfTestInternals() {
    uint8_t status = 0xFF;

    // A: when idle, write-in-progress (bit 0) and write-enable-latch (bit 1) are clear.
    if (readStatus(status) != EEPROM_OK) {
        return 101;
    }
    if ((status & 0x03) != 0) {
        return 102;
    }

    // B: sending write enable sets the latch (bit 1).
    if (writeEnable() != EEPROM_OK) {
        return 103;
    }
    if (readStatus(status) != EEPROM_OK || (status & 0x02) == 0) {
        return 104;
    }

    // C: start a write by hand, then read the status BEFORE waiting.
    //    The busy bit (bit 0) should be set while the chip is saving.
    const uint8_t msg[4] = {CMD_WRITE, 0x3F, 0x20, 0x5A};
    select();
    HAL_SPI_Transmit(spi, msg, 4, SPI_TIMEOUT_MS);
    deselect();

    if (readStatus(status) != EEPROM_OK) {
        return 105;
    }
    bool busyRightAfter = (status & STATUS_WIP_MASK) != 0;

    // D: a 0 ms timeout should give up while the chip is still busy.
    int timeoutResult = waitUntilReady(0);

    // E: a normal timeout waits it out successfully.
    if (waitUntilReady(WRITE_TIMEOUT_MS) != EEPROM_OK) {
        return 106;
    }

    // F: after the write finishes, both bits are clear again.
    if (readStatus(status) != EEPROM_OK || (status & 0x03) != 0) {
        return 107;
    }

    if (!busyRightAfter) {
        return 108;  // never saw the busy bit: wrong bit, or the write never started
    }
    if (timeoutResult != EEPROM_ERR_TIMEOUT) {
        return 109;  // the timeout path did not trigger
    }

    return 0;
}

#endif
// ---- SELFTEST END ----
