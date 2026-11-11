#pragma once

#include <cstdint>
#include "zp_error.h"
#include "param_setup.hpp"

class AttitudeManager;

class AMParamSetup : public IParamSetup {
   public:
    explicit AMParamSetup(AttitudeManager* am);
    ZP_Error loadAllParams() override;
    ZP_Error bindAllParamCallbacks() override;

   private:
    AttitudeManager* am;

    // Flightmode param callbacks
    #ifdef PLANE
    static ZP_Error updatePIDRollKp(AttitudeManager* ctx, float val);
    static ZP_Error updatePIDRollKi(AttitudeManager* ctx, float val);
    static ZP_Error updatePIDRollKd(AttitudeManager* ctx, float val);
    static ZP_Error updatePIDRollTau(AttitudeManager* ctx, float val);
    static ZP_Error updatePIDRollIMax(AttitudeManager* ctx, float val);
    static ZP_Error updatePIDRollFF(AttitudeManager* ctx, float val);
    static ZP_Error updatePIDPitchKp(AttitudeManager* ctx, float val);
    static ZP_Error updatePIDPitchKi(AttitudeManager* ctx, float val);
    static ZP_Error updatePIDPitchKd(AttitudeManager* ctx, float val);
    static ZP_Error updatePIDPitchTau(AttitudeManager* ctx, float val);
    static ZP_Error updatePIDPitchIMax(AttitudeManager* ctx, float val);
    static ZP_Error updatePIDPitchFF(AttitudeManager* ctx, float val);
    static ZP_Error updateKffRddrmix(AttitudeManager* ctx, float val);
    static ZP_Error updateRollLimitDeg(AttitudeManager* ctx, float val);
    static ZP_Error updatePitchLimMaxDeg(AttitudeManager* ctx, float val);
    static ZP_Error updatePitchLimMinDeg(AttitudeManager* ctx, float val);
    #endif
    #ifdef QUADCOPTER
    static ZP_Error updateRatePIDRollKp(AttitudeManager* ctx, float val);
    static ZP_Error updateRatePIDRollKi(AttitudeManager* ctx, float val);
    static ZP_Error updateRatePIDRollKd(AttitudeManager* ctx, float val);
    static ZP_Error updateRatePIDRollTau(AttitudeManager* ctx, float val);
    static ZP_Error updateRatePIDRollIMax(AttitudeManager* ctx, float val);
    static ZP_Error updateRatePIDPitchKp(AttitudeManager* ctx, float val);
    static ZP_Error updateRatePIDPitchKi(AttitudeManager* ctx, float val);
    static ZP_Error updateRatePIDPitchKd(AttitudeManager* ctx, float val);
    static ZP_Error updateRatePIDPitchTau(AttitudeManager* ctx, float val);
    static ZP_Error updateRatePIDPitchIMax(AttitudeManager* ctx, float val);
    static ZP_Error updateRatePIDYawKp(AttitudeManager* ctx, float val);
    static ZP_Error updateRatePIDYawKi(AttitudeManager* ctx, float val);
    static ZP_Error updateRatePIDYawKd(AttitudeManager* ctx, float val);
    static ZP_Error updateRatePIDYawTau(AttitudeManager* ctx, float val);
    static ZP_Error updateRatePIDYawIMax(AttitudeManager* ctx, float val);
    static ZP_Error updateRollPitchLimitRate(AttitudeManager* ctx, float val);
    static ZP_Error updateYawLimitRate(AttitudeManager* ctx, float val);

    static ZP_Error updateAngPIDRollKp(AttitudeManager* ctx, float val);
    static ZP_Error updateAngPIDRollKi(AttitudeManager* ctx, float val);
    static ZP_Error updateAngPIDRollKd(AttitudeManager* ctx, float val);
    static ZP_Error updateAngPIDRollTau(AttitudeManager* ctx, float val);
    static ZP_Error updateAngPIDRollIMax(AttitudeManager* ctx, float val);
    static ZP_Error updateAngPIDPitchKp(AttitudeManager* ctx, float val);
    static ZP_Error updateAngPIDPitchKi(AttitudeManager* ctx, float val);
    static ZP_Error updateAngPIDPitchKd(AttitudeManager* ctx, float val);
    static ZP_Error updateAngPIDPitchTau(AttitudeManager* ctx, float val);
    static ZP_Error updateAngPIDPitchIMax(AttitudeManager* ctx, float val);
    static ZP_Error updateRollPitchLimitAng(AttitudeManager* ctx, float val);
    static ZP_Error updateMotSpinMin(AttitudeManager* ctx, float val);
    static ZP_Error updateMotSpinMax(AttitudeManager* ctx, float val);
    static ZP_Error updateMotSpinArm(AttitudeManager* ctx, float val);
    #endif

    // FFT Harmonic Notch Filter param callbacks
    static ZP_Error updateHarmonicNotchEnabled(AttitudeManager* ctx, float val);
    static ZP_Error updateHarmonicNotchWindowSize(AttitudeManager* ctx, float val);
    static ZP_Error updateHarmonicNotchMinFreqHz(AttitudeManager* ctx, float val);
    static ZP_Error updateHarmonicNotchBandwidthHz(AttitudeManager* ctx, float val);
    static ZP_Error updateHarmonicNotchAttenuationDB(AttitudeManager* ctx, float val);
    static ZP_Error updateHarmonicNotchHarmonicsMask(AttitudeManager* ctx, float val);

    // Servo param callback helpers
    static ZP_Error setServoTrim(AttitudeManager* ctx, uint8_t ch, float val);
    static ZP_Error setServoMin(AttitudeManager* ctx, uint8_t ch, float val);
    static ZP_Error setServoMax(AttitudeManager* ctx, uint8_t ch, float val);
    static ZP_Error setServoReversed(AttitudeManager* ctx, uint8_t ch, float val);
    static ZP_Error setServoFunction(AttitudeManager* ctx, uint8_t ch, float val);

    // Compile-time: each instantiation is a distinct function pointer
    template <uint8_t Ch> static ZP_Error cbServoTrim(AttitudeManager* ctx, float v)     { return setServoTrim(ctx, Ch, v); }
    template <uint8_t Ch> static ZP_Error cbServoMin(AttitudeManager* ctx, float v)      { return setServoMin(ctx, Ch, v); }
    template <uint8_t Ch> static ZP_Error cbServoMax(AttitudeManager* ctx, float v)      { return setServoMax(ctx, Ch, v); }
    template <uint8_t Ch> static ZP_Error cbServoReversed(AttitudeManager* ctx, float v) { return setServoReversed(ctx, Ch, v); }
    template <uint8_t Ch> static ZP_Error cbServoFunction(AttitudeManager* ctx, float v) { return setServoFunction(ctx, Ch, v); }
};
