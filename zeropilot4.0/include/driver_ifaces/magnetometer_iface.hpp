#pragma once

#include <cstdint>

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

        virtual bool init() = 0;

        // Non blocking; isNew false until a sample completes.
        virtual MagData_t readData() = 0;
};
