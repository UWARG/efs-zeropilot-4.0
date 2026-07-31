#pragma once

#include <cstdint>

typedef struct {
    float x;             // uT, calibrated
    float y;             // uT, calibrated
    float z;             // uT, calibrated
    int16_t rawX;        // LSB
    int16_t rawY;        // LSB
    int16_t rawZ;        // LSB
    uint32_t timestamp;  // ms
    bool isNew;
} MagData_t;

typedef enum : uint8_t {
    MAG_CAL_STATE_IDLE = 0,
    MAG_CAL_STATE_COLLECTING,
    MAG_CAL_STATE_SUCCESS,
    MAG_CAL_STATE_FAILED,
} MagCalState_t;

typedef enum : uint8_t {
    MAG_CAL_ERR_NONE = 0,
    MAG_CAL_ERR_TOO_FEW_SAMPLES,
    MAG_CAL_ERR_POOR_COVERAGE,
    MAG_CAL_ERR_SINGULAR,
    MAG_CAL_ERR_IMPLAUSIBLE_FIELD,
} MagCalError_t;

typedef struct {
    MagCalState_t state;
    MagCalError_t error;
    uint32_t sampleCount;
    uint8_t progressPercent;
} MagCalStatus_t;

// corrected = softIron * (measured - hardIron)
typedef struct {
    float hardIron[3];    // uT
    float softIron[9];    // row major 3x3
    float fieldStrength;  // uT
} MagCalConstants_t;

class IMagnetometer {
    protected:
        IMagnetometer() = default;

    public:
        virtual ~IMagnetometer() = default;

        virtual bool init() = 0;

        // Non blocking; isNew false until a sample completes.
        virtual MagData_t readData() = 0;

        // On demand; driven by subsequent readData() calls.
        virtual void startCalibration() = 0;
        virtual void cancelCalibration() = 0;
        virtual MagCalStatus_t getCalibrationStatus() = 0;
        virtual const MagCalConstants_t &getCalibrationConstants() const = 0;
};
