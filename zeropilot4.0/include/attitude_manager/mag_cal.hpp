#pragma once

#include <cstdint>
#include "magnetometer_iface.hpp"

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

/**
 * Hard and soft iron calibration for any magnetometer.
 *
 * The correction and the ellipsoid fit that produces it depend only on the
 * field a sensor reports, never on how that sensor is wired or addressed, so
 * this lives in ZP core and drives an IMagnetometer through the interface.
 * Drivers are responsible only for reporting field in uT.
 *
 * Calibration is on demand: startCalibration() begins a collection window that
 * is advanced by subsequent update() calls, and the fit is solved once the
 * window closes. Timing is taken from sample timestamps, so no clock
 * dependency is needed.
 */
class MagCal {
    public:
        static constexpr uint32_t COLLECT_DURATION_MS = 30000;

        explicit MagCal(IMagnetometer *magDriver);

        /**
         * Reads the driver and returns the sample with the current correction
         * applied. Non blocking; isNew is false until a sample completes.
         * While collecting, the uncorrected field is fed to the fit.
         */
        MagData_t update();

        void startCalibration();
        void cancelCalibration();

        MagCalStatus_t getStatus() const;

        const MagCalConstants_t &getConstants() const;

        // For constants restored from NVM or ZP params rather than a live fit.
        void setConstants(const MagCalConstants_t &newConstants);
        void resetToDefaults();

    private:
        static constexpr uint32_t MIN_SAMPLES = 60;
        static constexpr float MIN_AXIS_SPAN_UT = 10.0f;
        static constexpr float MIN_SPAN_RATIO = 0.5f;
        static constexpr float MIN_FIELD_UT = 5.0f;
        static constexpr float MAX_FIELD_UT = 200.0f;

        IMagnetometer *magDriver;

        MagCalConstants_t constants;
        MagCalState_t state;
        MagCalError_t error;

        uint32_t startMs;
        uint32_t lastSampleMs;
        bool haveStartMs;
        uint32_t sampleCount;

        float normalMatrix[16];
        float normalVector[4];
        float axisMin[3];
        float axisMax[3];

        void resetAccumulators();
        void addSample(const float field[3], uint32_t nowMs);
        void applyCorrection(float field[3]) const;
        bool solve();
        static bool invert4x4(const float src[16], float dst[16]);
};
