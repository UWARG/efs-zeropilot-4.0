#pragma once

#include <cstdint>

#include "imu_datatypes.hpp"

class IIMU {
protected:
	IIMU() = default;

public:
    virtual ~IIMU() = default;
	virtual int init() = 0;
	virtual RawImu_t readRawData() = 0;
	virtual ScaledImu_t scaleIMUData(const RawImu_t &rawData) = 0;
};
