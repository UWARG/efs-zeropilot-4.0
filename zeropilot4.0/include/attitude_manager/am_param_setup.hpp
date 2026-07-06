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

    // Flightmode param callbacks
    #ifdef PLANE
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
    #endif
    #ifdef QUADCOPTER
    static bool updateRatePIDRollKp(AttitudeManager* ctx, float val);
    static bool updateRatePIDRollKi(AttitudeManager* ctx, float val);
    static bool updateRatePIDRollKd(AttitudeManager* ctx, float val);
    static bool updateRatePIDRollTau(AttitudeManager* ctx, float val);
    static bool updateRatePIDRollIMax(AttitudeManager* ctx, float val);
    static bool updateRatePIDPitchKp(AttitudeManager* ctx, float val);
    static bool updateRatePIDPitchKi(AttitudeManager* ctx, float val);
    static bool updateRatePIDPitchKd(AttitudeManager* ctx, float val);
    static bool updateRatePIDPitchTau(AttitudeManager* ctx, float val);
    static bool updateRatePIDPitchIMax(AttitudeManager* ctx, float val);
    static bool updateRatePIDYawKp(AttitudeManager* ctx, float val);
    static bool updateRatePIDYawKi(AttitudeManager* ctx, float val);
    static bool updateRatePIDYawKd(AttitudeManager* ctx, float val);
    static bool updateRatePIDYawTau(AttitudeManager* ctx, float val);
    static bool updateRatePIDYawIMax(AttitudeManager* ctx, float val);
    static bool updateRollLimitRate(AttitudeManager* ctx, float val);
    static bool updatePitchLimitRate(AttitudeManager* ctx, float val);
    static bool updateYawLimitRate(AttitudeManager* ctx, float val);

    static bool updateAngPIDRollKp(AttitudeManager* ctx, float val);
    static bool updateAngPIDRollKi(AttitudeManager* ctx, float val);
    static bool updateAngPIDRollKd(AttitudeManager* ctx, float val);
    static bool updateAngPIDRollTau(AttitudeManager* ctx, float val);
    static bool updateAngPIDRollIMax(AttitudeManager* ctx, float val);
    static bool updateAngPIDPitchKp(AttitudeManager* ctx, float val);
    static bool updateAngPIDPitchKi(AttitudeManager* ctx, float val);
    static bool updateAngPIDPitchKd(AttitudeManager* ctx, float val);
    static bool updateAngPIDPitchTau(AttitudeManager* ctx, float val);
    static bool updateAngPIDPitchIMax(AttitudeManager* ctx, float val);
    static bool updateRollLimitAng(AttitudeManager* ctx, float val);
    static bool updatePitchLimitAng(AttitudeManager* ctx, float val);
    #endif

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
