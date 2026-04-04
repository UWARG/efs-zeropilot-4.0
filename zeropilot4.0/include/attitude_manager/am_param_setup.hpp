#pragma once

#include <cstdint>

#include "param_setup.hpp"

class AttitudeManager;

class AMParamSetup : public IParamSetup {
   public:
    explicit AMParamSetup(AttitudeManager* am);
    void loadAllParams() override;
    void bindAllParamCallbacks() override;

   private:
    AttitudeManager* am;

    // FBWA param callbacks
    static bool updatePIDRollKp(AttitudeManager* ctx, float val);
    static bool updatePIDRollKi(AttitudeManager* ctx, float val);
    static bool updatePIDRollKd(AttitudeManager* ctx, float val);
    static bool updatePIDRollTau(AttitudeManager* ctx, float val);
    static bool updatePIDRollIMax(AttitudeManager* ctx, float val);
    static bool updatePIDPitchKp(AttitudeManager* ctx, float val);
    static bool updatePIDPitchKi(AttitudeManager* ctx, float val);
    static bool updatePIDPitchKd(AttitudeManager* ctx, float val);
    static bool updatePIDPitchTau(AttitudeManager* ctx, float val);
    static bool updatePIDPitchIMax(AttitudeManager* ctx, float val);
    static bool updateKffRddrmix(AttitudeManager* ctx, float val);
    static bool updateRollLimitDeg(AttitudeManager* ctx, float val);
    static bool updatePitchLimMaxDeg(AttitudeManager* ctx, float val);
    static bool updatePitchLimMinDeg(AttitudeManager* ctx, float val);

    // Servo param callback helpers
    static bool setServoTrim(AttitudeManager* ctx, uint8_t ch, float val);
    static bool setServoMin(AttitudeManager* ctx, uint8_t ch, float val);
    static bool setServoMax(AttitudeManager* ctx, uint8_t ch, float val);
    static bool setServoReversed(AttitudeManager* ctx, uint8_t ch, float val);
    static bool setServoFunction(AttitudeManager* ctx, uint8_t ch, float val);

    // Compile-time: each instantiation is a distinct function pointer
    template <uint8_t Ch> static bool cbServoTrim(AttitudeManager* ctx, float v)     { return setServoTrim(ctx, Ch, v); }
    template <uint8_t Ch> static bool cbServoMin(AttitudeManager* ctx, float v)      { return setServoMin(ctx, Ch, v); }
    template <uint8_t Ch> static bool cbServoMax(AttitudeManager* ctx, float v)      { return setServoMax(ctx, Ch, v); }
    template <uint8_t Ch> static bool cbServoReversed(AttitudeManager* ctx, float v) { return setServoReversed(ctx, Ch, v); }
    template <uint8_t Ch> static bool cbServoFunction(AttitudeManager* ctx, float v) { return setServoFunction(ctx, Ch, v); }
};
