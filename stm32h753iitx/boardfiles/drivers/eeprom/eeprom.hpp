#pragma once

// ---- SELFTEST BEGIN (temporary, delete before committing) ----
#define EEPROM_SELFTEST
// ---- SELFTEST END ----

#include "eeprom_iface.hpp"
#include "stm32h7xx_hal.h"

class EEPROM : public IEEPROM {
    public:
        EEPROM(SPI_HandleTypeDef *spi, GPIO_TypeDef *csPort, uint16_t csPin);

        /**
         * @brief reads bytes from the EEPROM into a buffer
         * @param address first address to read from (0 to 16383)
         * @param buffer where the bytes are stored, must hold at least length bytes
         * @param length number of bytes to read
         * @retval EEPROMResult: 0 on success
         */
        int read(uint16_t address, uint8_t *buffer, uint16_t length) override;

        /**
         * @brief writes bytes to the EEPROM, splitting at page boundaries
         * @param address first address to write to (0 to 16383)
         * @param data bytes to write
         * @param length number of bytes to write
         * @retval EEPROMResult: 0 on success
         */
        int write(uint16_t address, const uint8_t *data, uint16_t length) override;

    private:
        SPI_HandleTypeDef * const spi;
        GPIO_TypeDef * const csPort;
        const uint16_t csPin;

        /**
         * @brief pulls chip select low to start a transaction
         */
        void select();

        /**
         * @brief pulls chip select high to end a transaction
         */
        void deselect();

        /**
         * @brief sends the write enable command (its own full transaction)
         * @retval EEPROMResult: 0 on success
         */
        int writeEnable();

        /**
         * @brief reads the status register
         * @param status filled with the register value
         * @retval EEPROMResult: 0 on success
         */
        int readStatus(uint8_t &status);

        /**
         * @brief polls the status register until the write in progress bit clears
         * @param timeoutMs how long to wait before giving up
         * @retval EEPROMResult: 0 when ready, EEPROM_ERR_TIMEOUT if it never finished
         */
        int waitUntilReady(uint32_t timeoutMs);

// ---- SELFTEST BEGIN (temporary, delete before committing) ----
#ifdef EEPROM_SELFTEST
    public:
        /**
         * @brief runs every public function against the real chip
         * @retval 0 if all tests passed, otherwise the number of the first failing test
         */
        int selfTest();

        /**
         * @brief runs the private helpers against the real chip
         * @retval 0 if all checks passed, otherwise 101 or higher
         */
        int selfTestInternals();

        /**
         * @brief increments a counter stored in the chip, to prove data survives power cycles
         * @retval the new count, or a negative number on failure
         */
        int bootCounter();
#endif
// ---- SELFTEST END ----
};
