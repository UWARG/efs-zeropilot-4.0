#pragma once

#include <cstdint>
#include "error.h"

class ITelemLink {
protected:
	ITelemLink() = default;

public:
    virtual ~ITelemLink() = default;
    virtual ZP_ERROR_e transmit(const uint8_t* data, uint16_t size) = 0;
    virtual ZP_ERROR_e receive(uint16_t *received_size, uint8_t* buffer, uint16_t bufferSize) = 0;
};
