#pragma once

#include <cstdint>
#include "zp_error.h"
#include "param_setup.hpp"

class AttitudeManager;

class AMParamSetup : public IParamSetup {
   public:
    explicit AMParamSetup(AttitudeManager* am);
    ZP_ERROR_e loadAllParams() override;
    ZP_ERROR_e bindAllParamCallbacks() override;

   private:
    AttitudeManager* am;

    // FBWA param callbacks
    static ZP_ERROR_e updatePIDRollKp(AttitudeManager* ctx, float val);
    static ZP_ERROR_e updatePIDRollKi(AttitudeManager* ctx, float val);
    static ZP_ERROR_e updatePIDRollKd(AttitudeManager* ctx, float val);
    static ZP_ERROR_e updatePIDRollTau(AttitudeManager* ctx, float val);
    static ZP_ERROR_e updatePIDRollIMax(AttitudeManager* ctx, float val);
    static ZP_ERROR_e updatePIDPitchKp(AttitudeManager* ctx, float val);
    static ZP_ERROR_e updatePIDPitchKi(AttitudeManager* ctx, float val);
    static ZP_ERROR_e updatePIDPitchKd(AttitudeManager* ctx, float val);
    static ZP_ERROR_e updatePIDPitchTau(AttitudeManager* ctx, float val);
    static ZP_ERROR_e updatePIDPitchIMax(AttitudeManager* ctx, float val);
    static ZP_ERROR_e updateKffRddrmix(AttitudeManager* ctx, float val);
    static ZP_ERROR_e updateRollLimitDeg(AttitudeManager* ctx, float val);
    static ZP_ERROR_e updatePitchLimMaxDeg(AttitudeManager* ctx, float val);
    static ZP_ERROR_e updatePitchLimMinDeg(AttitudeManager* ctx, float val);

    // Servo param callback helpers
    static ZP_ERROR_e setServoTrim(AttitudeManager* ctx, uint8_t ch, float val);
    static ZP_ERROR_e setServoMin(AttitudeManager* ctx, uint8_t ch, float val);
    static ZP_ERROR_e setServoMax(AttitudeManager* ctx, uint8_t ch, float val);
    static ZP_ERROR_e setServoReversed(AttitudeManager* ctx, uint8_t ch, float val);
    static ZP_ERROR_e setServoFunction(AttitudeManager* ctx, uint8_t ch, float val);

    // Compile-time: each instantiation is a distinct function pointer
    template <uint8_t Ch> static ZP_ERROR_e cbServoTrim(AttitudeManager* ctx, float v)     { return setServoTrim(ctx, Ch, v); }
    template <uint8_t Ch> static ZP_ERROR_e cbServoMin(AttitudeManager* ctx, float v)      { return setServoMin(ctx, Ch, v); }
    template <uint8_t Ch> static ZP_ERROR_e cbServoMax(AttitudeManager* ctx, float v)      { return setServoMax(ctx, Ch, v); }
    template <uint8_t Ch> static ZP_ERROR_e cbServoReversed(AttitudeManager* ctx, float v) { return setServoReversed(ctx, Ch, v); }
    template <uint8_t Ch> static ZP_ERROR_e cbServoFunction(AttitudeManager* ctx, float v) { return setServoFunction(ctx, Ch, v); }
};
