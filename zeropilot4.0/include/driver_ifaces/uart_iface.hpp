#pragma once

#include <cstdint>
#include "zp_error.h"

class IUARTInterface {
    protected:
        IUARTInterface() = default;

    public:
        virtual ~IUARTInterface() = default;
        
        // huart Register Operations
        virtual ZP_Error transmit(uint8_t* pData, uint16_t size) = 0;
        virtual ZP_Error transmitDMA(uint8_t* pData, uint16_t size) = 0;
        virtual ZP_Error receive(uint8_t *pData, uint16_t size, uint32_t deadline) = 0;
        virtual ZP_Error restartDMA(uint8_t *pData, uint16_t size) = 0;
};
