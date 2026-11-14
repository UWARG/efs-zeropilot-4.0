#pragma once

#include <cstdint>

/**
 * Result codes returned by every IEEPROM function.
 * 0 means success, anything else is an error.
 */
enum EEPROMResult : int {
    EEPROM_OK = 0,
    EEPROM_ERR_SPI = 1,      // SPI transfer failed
    EEPROM_ERR_TIMEOUT = 2,  // chip never finished a write in time
    EEPROM_ERR_ARG = 3       // bad pointer, or address + length is past the end of memory
};

class IEEPROM {
    protected:
        IEEPROM() = default;

    public:
        virtual ~IEEPROM() = default;

        virtual int read(uint16_t address, uint8_t *buffer, uint16_t length) = 0;
        virtual int write(uint16_t address, const uint8_t *data, uint16_t length) = 0;
};
