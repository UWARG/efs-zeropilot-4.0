#pragma once

#include <cstdint>

class ITelemLink {
protected:
	ITelemLink() = default;

public:
    virtual ~ITelemLink() = default;
    virtual void transmit(const uint8_t* data, uint16_t size) = 0;
    virtual uint16_t receive(uint8_t* buffer, uint16_t bufferSize) = 0;
};
