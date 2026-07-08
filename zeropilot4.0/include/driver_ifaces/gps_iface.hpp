#pragma once

#include "gps_datatypes.hpp"

class IGPS {
protected:
    IGPS() = default;

public:
    virtual ~IGPS() = default;

    virtual GpsData_t readData() = 0;
};

struct rtcm_correction_data_t {
  uint8_t data[720];
  uint16_t len;
  bool newData;
};
