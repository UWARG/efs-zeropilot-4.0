#pragma once

#include <cstdint>

#include "imu_datatypes.hpp"

class IIMU {
protected:
	IIMU() = default;

public:
    virtual ~IIMU() = default;
	virtual ZP_ERROR_e init(int *value) = 0;
	virtual ZP_ERROR_e readRawData(RawImu_t *data) = 0;
	virtual ZP_ERROR_e scaleIMUData(const RawImu_t &rawData, ScaledImu_t *data) = 0;
};
