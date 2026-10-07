#pragma once

#include "gps.hpp"

class RTKGPS: public GPS {
public:
    HAL_StatusTypeDef sendCorrectionData(const uint8_t *data, uint16_t len);
};