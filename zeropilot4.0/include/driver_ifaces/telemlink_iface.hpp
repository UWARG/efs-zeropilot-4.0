#pragma once

#include <cstdint>
#include "zp_error.h"

class ITelemLink {
protected:
	ITelemLink() = default;

public:
    virtual ~ITelemLink() = default;
    virtual ZP_Error transmit(const uint8_t* data, uint16_t size) = 0;
    virtual ZP_Error receive(uint8_t* buffer, uint16_t bufferSize, uint16_t &received_size) = 0;
};
