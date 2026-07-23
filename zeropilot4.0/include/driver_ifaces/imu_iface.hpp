#pragma once

#include <cstdint>

#include "imu_datatypes.hpp"

class IIMU {
protected:
	IIMU() = default;

public:
    virtual ~IIMU() = default;
	virtual int init() = 0;
	virtual RawImuBatch_t readRawData() = 0;
	virtual ScaledImuBatch_t scaleIMUData(const RawImuBatch_t &rawDataBatch) = 0;
	virtual float getODRHz() = 0;
	virtual GyroStartupBias_t getGyroStartupBias() = 0;
};
