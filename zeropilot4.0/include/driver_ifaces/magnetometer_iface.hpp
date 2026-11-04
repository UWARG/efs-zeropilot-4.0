#pragma once

#include <cstdint>
#include "zp_error.h"

typedef struct {
    float x;             // uT, uncalibrated
    float y;             // uT, uncalibrated
    float z;             // uT, uncalibrated
    int16_t rawX;        // LSB
    int16_t rawY;        // LSB
    int16_t rawZ;        // LSB
    uint32_t timestamp;  // ms
    bool isNew;
} MagData_t;

/**
 * A magnetometer reports field only. Hard and soft iron calibration is sensor
 * agnostic and lives in ZP core, in MagCal, which drives this interface.
 */
class IMagnetometer {
    protected:
        IMagnetometer() = default;

    public:
        virtual ~IMagnetometer() = default;

        virtual ZP_Error init() = 0;

        /*
        @brief Non blocking. isNew is false until a sample completes, which is not an error
        @retval a bus or sensor fault seen since the last call
        */
        virtual ZP_Error readData(MagData_t &data) = 0;
};
