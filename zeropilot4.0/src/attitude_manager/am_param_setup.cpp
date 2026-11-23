#include "am_param_setup.hpp"
#include "attitude_manager.hpp"
#include "zp_params.hpp"
#include "motor_functions.hpp"
#include "unit_conversions.hpp"

static inline int usToPercent(float us) {
    return static_cast<int>((us - 1000.0f) / 10.0f);
}

static inline float readParam(ZP_Error &result, ZP_PARAM_ID id) {
    float value = 0.0f;
    result |= ZP_PARAM::get(id, value);
    return value;
}

AMParamSetup::AMParamSetup(AttitudeManager* am) : am(am) {}

ZP_Error AMParamSetup::loadAllParams() {
    ZP_Error result = ZP_ERROR_OK;

    #ifdef PLANE
    // FBWA params
    result |= am->fbwaCLAW.setRollPIDConstants(
        readParam(result, ZP_PARAM_ID::RLL2SRV_P),
        readParam(result, ZP_PARAM_ID::RLL2SRV_I),
        readParam(result, ZP_PARAM_ID::RLL2SRV_D),
        readParam(result, ZP_PARAM_ID::RLL2SRV_TAU),
        readParam(result, ZP_PARAM_ID::RLL2SRV_IMAX)
    );
    result |= am->fbwaCLAW.setPitchPIDConstants(
        readParam(result, ZP_PARAM_ID::PTCH2SRV_P),
        readParam(result, ZP_PARAM_ID::PTCH2SRV_I),
        readParam(result, ZP_PARAM_ID::PTCH2SRV_D),
        readParam(result, ZP_PARAM_ID::PTCH2SRV_TAU),
        readParam(result, ZP_PARAM_ID::PTCH2SRV_IMAX)
    );
    am->fbwaCLAW.setRollFFConstant(readParam(result, ZP_PARAM_ID::RLL2SRV_FF));
    am->fbwaCLAW.setPitchFFConstant(readParam(result, ZP_PARAM_ID::PTCH2SRV_FF));
    result |= am->fbwaCLAW.setYawRudderMixingConstant(readParam(result, ZP_PARAM_ID::KFF_RDDRMIX));
    result |= am->fbwaCLAW.setRollLimitDeg(readParam(result, ZP_PARAM_ID::ROLL_LIMIT_DEG));
    result |= am->fbwaCLAW.setPitchLimitMaxDeg(readParam(result, ZP_PARAM_ID::PTCH_LIM_MAX_DEG));
    result |= am->fbwaCLAW.setPitchLimitMinDeg(readParam(result, ZP_PARAM_ID::PTCH_LIM_MIN_DEG));
    #endif
    #ifdef QUADCOPTER
    // ACRO params
    am->acroCLAW.setRollPIDConstants(
        readParam(result, ZP_PARAM_ID::ATC_RAT_RLL_P),
        readParam(result, ZP_PARAM_ID::ATC_RAT_RLL_I),
        readParam(result, ZP_PARAM_ID::ATC_RAT_RLL_D),
        readParam(result, ZP_PARAM_ID::ATC_RAT_RLL_TAU),
        readParam(result, ZP_PARAM_ID::ATC_RAT_RLL_IMAX)
    );
    am->acroCLAW.setPitchPIDConstants(
        readParam(result, ZP_PARAM_ID::ATC_RAT_PIT_P),
        readParam(result, ZP_PARAM_ID::ATC_RAT_PIT_I),
        readParam(result, ZP_PARAM_ID::ATC_RAT_PIT_D),
        readParam(result, ZP_PARAM_ID::ATC_RAT_PIT_TAU),
        readParam(result, ZP_PARAM_ID::ATC_RAT_PIT_IMAX)
    );
    am->acroCLAW.setYawPIDConstants(
        readParam(result, ZP_PARAM_ID::ATC_RAT_YAW_P),
        readParam(result, ZP_PARAM_ID::ATC_RAT_YAW_I),
        readParam(result, ZP_PARAM_ID::ATC_RAT_YAW_D),
        readParam(result, ZP_PARAM_ID::ATC_RAT_YAW_TAU),
        readParam(result, ZP_PARAM_ID::ATC_RAT_YAW_IMAX)
    );
    am->acroCLAW.setRollLimitRate(ZP_UNITS::deg2rad(readParam(result, ZP_PARAM_ID::ACRO_RP_RATE)));
    am->acroCLAW.setPitchLimitRate(ZP_UNITS::deg2rad(readParam(result, ZP_PARAM_ID::ACRO_RP_RATE)));
    am->acroCLAW.setYawLimitRate(ZP_UNITS::deg2rad(readParam(result, ZP_PARAM_ID::ACRO_Y_RATE)));

    // Stabilize params
    am->stabilizeCLAW.setRollPIDConstants(
        readParam(result, ZP_PARAM_ID::ATC_ANG_RLL_P),
        readParam(result, ZP_PARAM_ID::ATC_ANG_RLL_I),
        readParam(result, ZP_PARAM_ID::ATC_ANG_RLL_D),
        readParam(result, ZP_PARAM_ID::ATC_ANG_RLL_TAU),
        readParam(result, ZP_PARAM_ID::ATC_ANG_RLL_IMAX)
    );
    am->stabilizeCLAW.setPitchPIDConstants(
        readParam(result, ZP_PARAM_ID::ATC_ANG_PTCH_P),
        readParam(result, ZP_PARAM_ID::ATC_ANG_PTCH_I),
        readParam(result, ZP_PARAM_ID::ATC_ANG_PTCH_D),
        readParam(result, ZP_PARAM_ID::ATC_ANG_PTCH_TAU),
        readParam(result, ZP_PARAM_ID::ATC_ANG_PTCH_IMAX)
    );
    am->stabilizeCLAW.setRollPitchLimitAngle(readParam(result, ZP_PARAM_ID::ATC_ANGLE_MAX));
    am->motSpinMin = readParam(result, ZP_PARAM_ID::MOT_SPIN_MIN);
    am->motSpinMax = readParam(result, ZP_PARAM_ID::MOT_SPIN_MAX);
    am->motSpinArm = readParam(result, ZP_PARAM_ID::MOT_SPIN_ARM);
    #endif

    // FFT Harmonic Notch Filter params 
    am->harmonicNotchConfig.enabled = readParam(result, ZP_PARAM_ID::FFT_ENABLE);
    am->harmonicNotchConfig.fftWindowSize = readParam(result, ZP_PARAM_ID::FFT_WINDOW_LEN);
    am->harmonicNotchConfig.minFreqHz = readParam(result, ZP_PARAM_ID::FFT_MINHZ);
    am->harmonicNotchConfig.bandwidthHz = readParam(result, ZP_PARAM_ID::INS_HNTCH_BW);
    am->harmonicNotchConfig.attenuationDB = readParam(result, ZP_PARAM_ID::INS_HNTCH_ATT);
    am->harmonicNotchConfig.harmonicsMask = readParam(result, ZP_PARAM_ID::INS_HNTCH_HMNCS);

    // Servo params
    auto loadMotor = [&](uint8_t ch, ZP_PARAM_ID trim, ZP_PARAM_ID min, ZP_PARAM_ID max, ZP_PARAM_ID rev, ZP_PARAM_ID func) {
        if (ch >= am->mainMotorGroup->motorCount) return;
        MotorInstance_t* m = &am->mainMotorGroup->motors[ch];
        m->trim       = usToPercent(readParam(result, trim));
        m->min        = usToPercent(readParam(result, min));
        m->max        = usToPercent(readParam(result, max));
        m->isInverted = static_cast<int>(readParam(result, rev)) != 0;
        m->function   = static_cast<MotorFunction_e>(static_cast<int16_t>(readParam(result, func)));
    };
    loadMotor(0,  ZP_PARAM_ID::SERVO1_TRIM,  ZP_PARAM_ID::SERVO1_MIN,  ZP_PARAM_ID::SERVO1_MAX,  ZP_PARAM_ID::SERVO1_REVERSED,  ZP_PARAM_ID::SERVO1_FUNCTION);
    loadMotor(1,  ZP_PARAM_ID::SERVO2_TRIM,  ZP_PARAM_ID::SERVO2_MIN,  ZP_PARAM_ID::SERVO2_MAX,  ZP_PARAM_ID::SERVO2_REVERSED,  ZP_PARAM_ID::SERVO2_FUNCTION);
    loadMotor(2,  ZP_PARAM_ID::SERVO3_TRIM,  ZP_PARAM_ID::SERVO3_MIN,  ZP_PARAM_ID::SERVO3_MAX,  ZP_PARAM_ID::SERVO3_REVERSED,  ZP_PARAM_ID::SERVO3_FUNCTION);
    loadMotor(3,  ZP_PARAM_ID::SERVO4_TRIM,  ZP_PARAM_ID::SERVO4_MIN,  ZP_PARAM_ID::SERVO4_MAX,  ZP_PARAM_ID::SERVO4_REVERSED,  ZP_PARAM_ID::SERVO4_FUNCTION);
    loadMotor(4,  ZP_PARAM_ID::SERVO5_TRIM,  ZP_PARAM_ID::SERVO5_MIN,  ZP_PARAM_ID::SERVO5_MAX,  ZP_PARAM_ID::SERVO5_REVERSED,  ZP_PARAM_ID::SERVO5_FUNCTION);
    loadMotor(5,  ZP_PARAM_ID::SERVO6_TRIM,  ZP_PARAM_ID::SERVO6_MIN,  ZP_PARAM_ID::SERVO6_MAX,  ZP_PARAM_ID::SERVO6_REVERSED,  ZP_PARAM_ID::SERVO6_FUNCTION);
    loadMotor(6,  ZP_PARAM_ID::SERVO7_TRIM,  ZP_PARAM_ID::SERVO7_MIN,  ZP_PARAM_ID::SERVO7_MAX,  ZP_PARAM_ID::SERVO7_REVERSED,  ZP_PARAM_ID::SERVO7_FUNCTION);
    loadMotor(7,  ZP_PARAM_ID::SERVO8_TRIM,  ZP_PARAM_ID::SERVO8_MIN,  ZP_PARAM_ID::SERVO8_MAX,  ZP_PARAM_ID::SERVO8_REVERSED,  ZP_PARAM_ID::SERVO8_FUNCTION);
    loadMotor(8,  ZP_PARAM_ID::SERVO9_TRIM,  ZP_PARAM_ID::SERVO9_MIN,  ZP_PARAM_ID::SERVO9_MAX,  ZP_PARAM_ID::SERVO9_REVERSED,  ZP_PARAM_ID::SERVO9_FUNCTION);
    loadMotor(9,  ZP_PARAM_ID::SERVO10_TRIM, ZP_PARAM_ID::SERVO10_MIN, ZP_PARAM_ID::SERVO10_MAX, ZP_PARAM_ID::SERVO10_REVERSED, ZP_PARAM_ID::SERVO10_FUNCTION);
    loadMotor(10, ZP_PARAM_ID::SERVO11_TRIM, ZP_PARAM_ID::SERVO11_MIN, ZP_PARAM_ID::SERVO11_MAX, ZP_PARAM_ID::SERVO11_REVERSED, ZP_PARAM_ID::SERVO11_FUNCTION);
    loadMotor(11, ZP_PARAM_ID::SERVO12_TRIM, ZP_PARAM_ID::SERVO12_MIN, ZP_PARAM_ID::SERVO12_MAX, ZP_PARAM_ID::SERVO12_REVERSED, ZP_PARAM_ID::SERVO12_FUNCTION);

    return result;
}

// Macro to bind all 5 fields for a single servo channel
#define AM_PARAM_SETUP_BIND_SERVO_CB(N) \
    result |= ZP_PARAM::bindCallback(ZP_PARAM_ID::SERVO##N##_TRIM,     am, cbServoTrim<N-1>);     \
    result |= ZP_PARAM::bindCallback(ZP_PARAM_ID::SERVO##N##_MIN,      am, cbServoMin<N-1>);      \
    result |= ZP_PARAM::bindCallback(ZP_PARAM_ID::SERVO##N##_MAX,      am, cbServoMax<N-1>);      \
    result |= ZP_PARAM::bindCallback(ZP_PARAM_ID::SERVO##N##_REVERSED, am, cbServoReversed<N-1>); \
    result |= ZP_PARAM::bindCallback(ZP_PARAM_ID::SERVO##N##_FUNCTION, am, cbServoFunction<N-1>);

ZP_Error AMParamSetup::bindAllParamCallbacks() {
    ZP_Error result = ZP_ERROR_OK;

    // FBWA
    #ifdef PLANE
    result |= ZP_PARAM::bindCallback(ZP_PARAM_ID::RLL2SRV_P,           am, updatePIDRollKp);
    result |= ZP_PARAM::bindCallback(ZP_PARAM_ID::RLL2SRV_I,           am, updatePIDRollKi);
    result |= ZP_PARAM::bindCallback(ZP_PARAM_ID::RLL2SRV_D,           am, updatePIDRollKd);
    result |= ZP_PARAM::bindCallback(ZP_PARAM_ID::RLL2SRV_TAU,         am, updatePIDRollTau);
    result |= ZP_PARAM::bindCallback(ZP_PARAM_ID::RLL2SRV_IMAX,        am, updatePIDRollIMax);
    result |= ZP_PARAM::bindCallback(ZP_PARAM_ID::RLL2SRV_FF,          am, updatePIDRollFF);
    result |= ZP_PARAM::bindCallback(ZP_PARAM_ID::PTCH2SRV_P,          am, updatePIDPitchKp);
    result |= ZP_PARAM::bindCallback(ZP_PARAM_ID::PTCH2SRV_I,          am, updatePIDPitchKi);
    result |= ZP_PARAM::bindCallback(ZP_PARAM_ID::PTCH2SRV_D,          am, updatePIDPitchKd);
    result |= ZP_PARAM::bindCallback(ZP_PARAM_ID::PTCH2SRV_TAU,        am, updatePIDPitchTau);
    result |= ZP_PARAM::bindCallback(ZP_PARAM_ID::PTCH2SRV_IMAX,       am, updatePIDPitchIMax);
    result |= ZP_PARAM::bindCallback(ZP_PARAM_ID::PTCH2SRV_FF,         am, updatePIDPitchFF);
    result |= ZP_PARAM::bindCallback(ZP_PARAM_ID::KFF_RDDRMIX,         am, updateKffRddrmix);
    result |= ZP_PARAM::bindCallback(ZP_PARAM_ID::ROLL_LIMIT_DEG,      am, updateRollLimitDeg);
    result |= ZP_PARAM::bindCallback(ZP_PARAM_ID::PTCH_LIM_MAX_DEG,    am, updatePitchLimMaxDeg);
    result |= ZP_PARAM::bindCallback(ZP_PARAM_ID::PTCH_LIM_MIN_DEG,    am, updatePitchLimMinDeg);
    #endif
    #ifdef QUADCOPTER
    result |= ZP_PARAM::bindCallback(ZP_PARAM_ID::ATC_RAT_RLL_P,       am, updateRatePIDRollKp);
    result |= ZP_PARAM::bindCallback(ZP_PARAM_ID::ATC_RAT_RLL_I,       am, updateRatePIDRollKi);
    result |= ZP_PARAM::bindCallback(ZP_PARAM_ID::ATC_RAT_RLL_D,       am, updateRatePIDRollKd);
    result |= ZP_PARAM::bindCallback(ZP_PARAM_ID::ATC_RAT_RLL_TAU,     am, updateRatePIDRollTau);
    result |= ZP_PARAM::bindCallback(ZP_PARAM_ID::ATC_RAT_RLL_IMAX,    am, updateRatePIDRollIMax);
    result |= ZP_PARAM::bindCallback(ZP_PARAM_ID::ATC_RAT_PIT_P,       am, updateRatePIDPitchKp);
    result |= ZP_PARAM::bindCallback(ZP_PARAM_ID::ATC_RAT_PIT_I,       am, updateRatePIDPitchKi);
    result |= ZP_PARAM::bindCallback(ZP_PARAM_ID::ATC_RAT_PIT_D,       am, updateRatePIDPitchKd);
    result |= ZP_PARAM::bindCallback(ZP_PARAM_ID::ATC_RAT_PIT_TAU,     am, updateRatePIDPitchTau);
    result |= ZP_PARAM::bindCallback(ZP_PARAM_ID::ATC_RAT_PIT_IMAX,    am, updateRatePIDPitchIMax);
    result |= ZP_PARAM::bindCallback(ZP_PARAM_ID::ATC_RAT_YAW_P,       am, updateRatePIDYawKp);
    result |= ZP_PARAM::bindCallback(ZP_PARAM_ID::ATC_RAT_YAW_I,       am, updateRatePIDYawKi);
    result |= ZP_PARAM::bindCallback(ZP_PARAM_ID::ATC_RAT_YAW_D,       am, updateRatePIDYawKd);
    result |= ZP_PARAM::bindCallback(ZP_PARAM_ID::ATC_RAT_YAW_TAU,     am, updateRatePIDYawTau);
    result |= ZP_PARAM::bindCallback(ZP_PARAM_ID::ATC_RAT_YAW_IMAX,    am, updateRatePIDYawIMax);
    result |= ZP_PARAM::bindCallback(ZP_PARAM_ID::ACRO_RP_RATE,        am, updateRollPitchLimitRate);
    result |= ZP_PARAM::bindCallback(ZP_PARAM_ID::ACRO_Y_RATE,         am, updateYawLimitRate);

    result |= ZP_PARAM::bindCallback(ZP_PARAM_ID::ATC_ANG_RLL_P,       am, updateAngPIDRollKp);
    result |= ZP_PARAM::bindCallback(ZP_PARAM_ID::ATC_ANG_RLL_I,       am, updateAngPIDRollKi);
    result |= ZP_PARAM::bindCallback(ZP_PARAM_ID::ATC_ANG_RLL_D,       am, updateAngPIDRollKd);
    result |= ZP_PARAM::bindCallback(ZP_PARAM_ID::ATC_ANG_RLL_TAU,     am, updateAngPIDRollTau);
    result |= ZP_PARAM::bindCallback(ZP_PARAM_ID::ATC_ANG_RLL_IMAX,    am, updateAngPIDRollIMax);
    result |= ZP_PARAM::bindCallback(ZP_PARAM_ID::ATC_ANG_PTCH_P,      am, updateAngPIDPitchKp);
    result |= ZP_PARAM::bindCallback(ZP_PARAM_ID::ATC_ANG_PTCH_I,      am, updateAngPIDPitchKi);
    result |= ZP_PARAM::bindCallback(ZP_PARAM_ID::ATC_ANG_PTCH_D,      am, updateAngPIDPitchKd);
    result |= ZP_PARAM::bindCallback(ZP_PARAM_ID::ATC_ANG_PTCH_TAU,    am, updateAngPIDPitchTau);
    result |= ZP_PARAM::bindCallback(ZP_PARAM_ID::ATC_ANG_PTCH_IMAX,   am, updateAngPIDPitchIMax);
    result |= ZP_PARAM::bindCallback(ZP_PARAM_ID::ATC_ANGLE_MAX,       am, updateRollPitchLimitAng);
    result |= ZP_PARAM::bindCallback(ZP_PARAM_ID::MOT_SPIN_MIN,        am, updateMotSpinMin);
    result |= ZP_PARAM::bindCallback(ZP_PARAM_ID::MOT_SPIN_MAX,        am, updateMotSpinMax);
    result |= ZP_PARAM::bindCallback(ZP_PARAM_ID::MOT_SPIN_ARM,        am, updateMotSpinArm);
    #endif

    // FFT Harmonic Notch Filter params
    result |= ZP_PARAM::bindCallback(ZP_PARAM_ID::FFT_ENABLE,          am, updateHarmonicNotchEnabled);
    result |= ZP_PARAM::bindCallback(ZP_PARAM_ID::FFT_WINDOW_LEN,      am, updateHarmonicNotchWindowSize);
    result |= ZP_PARAM::bindCallback(ZP_PARAM_ID::FFT_MINHZ,           am, updateHarmonicNotchMinFreqHz);
    result |= ZP_PARAM::bindCallback(ZP_PARAM_ID::INS_HNTCH_BW,        am, updateHarmonicNotchBandwidthHz);
    result |= ZP_PARAM::bindCallback(ZP_PARAM_ID::INS_HNTCH_ATT,       am, updateHarmonicNotchAttenuationDB);
    result |= ZP_PARAM::bindCallback(ZP_PARAM_ID::INS_HNTCH_HMNCS,     am, updateHarmonicNotchHarmonicsMask);

    // Servo params: each AM_PARAM_SETUP_BIND_SERVO_CB expands to 5 bindCallback calls
    AM_PARAM_SETUP_BIND_SERVO_CB(1)
    AM_PARAM_SETUP_BIND_SERVO_CB(2)
    AM_PARAM_SETUP_BIND_SERVO_CB(3)
    AM_PARAM_SETUP_BIND_SERVO_CB(4)
    AM_PARAM_SETUP_BIND_SERVO_CB(5)
    AM_PARAM_SETUP_BIND_SERVO_CB(6)
    AM_PARAM_SETUP_BIND_SERVO_CB(7)
    AM_PARAM_SETUP_BIND_SERVO_CB(8)
    AM_PARAM_SETUP_BIND_SERVO_CB(9)
    AM_PARAM_SETUP_BIND_SERVO_CB(10)
    AM_PARAM_SETUP_BIND_SERVO_CB(11)
    AM_PARAM_SETUP_BIND_SERVO_CB(12)

    return result;
}

#undef AM_PARAM_SETUP_BIND_SERVO_CB

#ifdef PLANE
// FBWA callbacks
ZP_Error AMParamSetup::updatePIDRollKp(AttitudeManager* ctx, float val) {
    if (val < 0.0f) return ZP_ERROR_INVALID_ARG;
    PID* pid = nullptr;
    ZP_Error result = ctx->fbwaCLAW.getRollPID(pid);
    if (result == ZP_ERROR_OK) pid->setKp(val);
    return result;
}
ZP_Error AMParamSetup::updatePIDRollKi(AttitudeManager* ctx, float val) {
    if (val < 0.0f) return ZP_ERROR_INVALID_ARG;
    PID* pid = nullptr;
    ZP_Error result = ctx->fbwaCLAW.getRollPID(pid);
    if (result == ZP_ERROR_OK) pid->setKi(val);
    return result;
}
ZP_Error AMParamSetup::updatePIDRollKd(AttitudeManager* ctx, float val) {
    if (val < 0.0f) return ZP_ERROR_INVALID_ARG;
    PID* pid = nullptr;
    ZP_Error result = ctx->fbwaCLAW.getRollPID(pid);
    if (result == ZP_ERROR_OK) pid->setKd(val);
    return result;
}
ZP_Error AMParamSetup::updatePIDRollTau(AttitudeManager* ctx, float val) {
    if (val < 0.0f) return ZP_ERROR_INVALID_ARG;
    PID* pid = nullptr;
    ZP_Error result = ctx->fbwaCLAW.getRollPID(pid);
    if (result == ZP_ERROR_OK) pid->setTau(val);
    return result;
}
ZP_Error AMParamSetup::updatePIDRollIMax(AttitudeManager* ctx, float val) {
    if (val < 0.0f || val > 100.0f) return ZP_ERROR_INVALID_ARG;
    PID* pid = nullptr;
    ZP_Error result = ctx->fbwaCLAW.getRollPID(pid);
    if (result == ZP_ERROR_OK) {
        pid->setIntegralMinLimPct(static_cast<uint8_t>(val));
        pid->setIntegralMaxLimPct(static_cast<uint8_t>(val));
    }
    return result;
}
ZP_Error AMParamSetup::updatePIDRollFF(AttitudeManager* ctx, float val) {
    if (val < 0.0f) return ZP_ERROR_INVALID_ARG;
    ctx->fbwaCLAW.setRollFFConstant(val);
    return ZP_ERROR_OK;
}
ZP_Error AMParamSetup::updatePIDPitchKp(AttitudeManager* ctx, float val) {
    if (val < 0.0f) return ZP_ERROR_INVALID_ARG;
    PID* pid = nullptr;
    ZP_Error result = ctx->fbwaCLAW.getPitchPID(pid);
    if (result == ZP_ERROR_OK) pid->setKp(val);
    return result;
}
ZP_Error AMParamSetup::updatePIDPitchKi(AttitudeManager* ctx, float val) {
    if (val < 0.0f) return ZP_ERROR_INVALID_ARG;
    PID* pid = nullptr;
    ZP_Error result = ctx->fbwaCLAW.getPitchPID(pid);
    if (result == ZP_ERROR_OK) pid->setKi(val);
    return result;
}
ZP_Error AMParamSetup::updatePIDPitchKd(AttitudeManager* ctx, float val) {
    if (val < 0.0f) return ZP_ERROR_INVALID_ARG;
    PID* pid = nullptr;
    ZP_Error result = ctx->fbwaCLAW.getPitchPID(pid);
    if (result == ZP_ERROR_OK) pid->setKd(val);
    return result;
}
ZP_Error AMParamSetup::updatePIDPitchTau(AttitudeManager* ctx, float val) {
    if (val < 0.0f) return ZP_ERROR_INVALID_ARG;
    PID* pid = nullptr;
    ZP_Error result = ctx->fbwaCLAW.getPitchPID(pid);
    if (result == ZP_ERROR_OK) pid->setTau(val);
    return result;
}
ZP_Error AMParamSetup::updatePIDPitchIMax(AttitudeManager* ctx, float val) {
    if (val < 0.0f || val > 100.0f) return ZP_ERROR_INVALID_ARG;
    PID* pid = nullptr;
    ZP_Error result = ctx->fbwaCLAW.getPitchPID(pid);
    if (result == ZP_ERROR_OK) {
        pid->setIntegralMinLimPct(static_cast<uint8_t>(val));
        pid->setIntegralMaxLimPct(static_cast<uint8_t>(val));
    }
    return result;
}
ZP_Error AMParamSetup::updatePIDPitchFF(AttitudeManager* ctx, float val) {
    if (val < 0.0f) return ZP_ERROR_INVALID_ARG;
    ctx->fbwaCLAW.setPitchFFConstant(val);
    return ZP_ERROR_OK;
}
ZP_Error AMParamSetup::updateKffRddrmix(AttitudeManager* ctx, float val) {
    if (val < 0.0f || val > 1.0f) return ZP_ERROR_INVALID_ARG;
    return ctx->fbwaCLAW.setYawRudderMixingConstant(val);
}
ZP_Error AMParamSetup::updateRollLimitDeg(AttitudeManager* ctx, float val) {
    if (val < 0.0f || val > 90.0f) return ZP_ERROR_INVALID_ARG;
    return ctx->fbwaCLAW.setRollLimitDeg(val);
}
ZP_Error AMParamSetup::updatePitchLimMaxDeg(AttitudeManager* ctx, float val) {
    if (val < 0.0f || val > 90.0f) return ZP_ERROR_INVALID_ARG;
    return ctx->fbwaCLAW.setPitchLimitMaxDeg(val);
}
ZP_Error AMParamSetup::updatePitchLimMinDeg(AttitudeManager* ctx, float val) {
    if (val < -90.0f || val > 0.0f) return ZP_ERROR_INVALID_ARG;
    return ctx->fbwaCLAW.setPitchLimitMinDeg(val);
}
#endif
#ifdef QUADCOPTER
// Acro callbacks
ZP_Error AMParamSetup::updateRatePIDRollKp(AttitudeManager* ctx, float val) {
    if (val < 0.0f) return ZP_ERROR_INVALID_ARG;
    ctx->acroCLAW.getRollPID()->setKp(val);
    return ZP_ERROR_OK;
}
ZP_Error AMParamSetup::updateRatePIDRollKi(AttitudeManager* ctx, float val) {
    if (val < 0.0f) return ZP_ERROR_INVALID_ARG;
    ctx->acroCLAW.getRollPID()->setKi(val);
    return ZP_ERROR_OK;
}
ZP_Error AMParamSetup::updateRatePIDRollKd(AttitudeManager* ctx, float val) {
    if (val < 0.0f) return ZP_ERROR_INVALID_ARG;
    ctx->acroCLAW.getRollPID()->setKd(val);
    return ZP_ERROR_OK;
}
ZP_Error AMParamSetup::updateRatePIDRollTau(AttitudeManager* ctx, float val) {
    if (val < 0.0f) return ZP_ERROR_INVALID_ARG;
    ctx->acroCLAW.getRollPID()->setTau(val);
    return ZP_ERROR_OK;
}
ZP_Error AMParamSetup::updateRatePIDRollIMax(AttitudeManager* ctx, float val) {
    if (val < 0.0f || val > 100.0f) return ZP_ERROR_INVALID_ARG;
    ctx->acroCLAW.getRollPID()->setIntegralMinLimPct(static_cast<uint8_t>(val));
    ctx->acroCLAW.getRollPID()->setIntegralMaxLimPct(static_cast<uint8_t>(val));
    return ZP_ERROR_OK;
}
ZP_Error AMParamSetup::updateRatePIDPitchKp(AttitudeManager* ctx, float val) {
    if (val < 0.0f) return ZP_ERROR_INVALID_ARG;
    ctx->acroCLAW.getPitchPID()->setKp(val);
    return ZP_ERROR_OK;
}
ZP_Error AMParamSetup::updateRatePIDPitchKi(AttitudeManager* ctx, float val) {
    if (val < 0.0f) return ZP_ERROR_INVALID_ARG;
    ctx->acroCLAW.getPitchPID()->setKi(val);
    return ZP_ERROR_OK;
}
ZP_Error AMParamSetup::updateRatePIDPitchKd(AttitudeManager* ctx, float val) {
    if (val < 0.0f) return ZP_ERROR_INVALID_ARG;
    ctx->acroCLAW.getPitchPID()->setKd(val);
    return ZP_ERROR_OK;
}
ZP_Error AMParamSetup::updateRatePIDPitchTau(AttitudeManager* ctx, float val) {
    if (val < 0.0f) return ZP_ERROR_INVALID_ARG;
    ctx->acroCLAW.getPitchPID()->setTau(val);
    return ZP_ERROR_OK;
}
ZP_Error AMParamSetup::updateRatePIDPitchIMax(AttitudeManager* ctx, float val) {
    if (val < 0.0f || val > 100.0f) return ZP_ERROR_INVALID_ARG;
    ctx->acroCLAW.getPitchPID()->setIntegralMinLimPct(static_cast<uint8_t>(val));
    ctx->acroCLAW.getPitchPID()->setIntegralMaxLimPct(static_cast<uint8_t>(val));
    return ZP_ERROR_OK;
}
ZP_Error AMParamSetup::updateRatePIDYawKp(AttitudeManager* ctx, float val) {
    if (val < 0.0f) return ZP_ERROR_INVALID_ARG;
    ctx->acroCLAW.getYawPID()->setKp(val);
    return ZP_ERROR_OK;
}
ZP_Error AMParamSetup::updateRatePIDYawKi(AttitudeManager* ctx, float val) {
    if (val < 0.0f) return ZP_ERROR_INVALID_ARG;
    ctx->acroCLAW.getYawPID()->setKi(val);
    return ZP_ERROR_OK;
}
ZP_Error AMParamSetup::updateRatePIDYawKd(AttitudeManager* ctx, float val) {
    if (val < 0.0f) return ZP_ERROR_INVALID_ARG;
    ctx->acroCLAW.getYawPID()->setKd(val);
    return ZP_ERROR_OK;
}
ZP_Error AMParamSetup::updateRatePIDYawTau(AttitudeManager* ctx, float val) {
    if (val < 0.0f) return ZP_ERROR_INVALID_ARG;
    ctx->acroCLAW.getYawPID()->setTau(val);
    return ZP_ERROR_OK;
}
ZP_Error AMParamSetup::updateRatePIDYawIMax(AttitudeManager* ctx, float val) {
    if (val < 0.0f || val > 100.0f) return ZP_ERROR_INVALID_ARG;
    ctx->acroCLAW.getYawPID()->setIntegralMinLimPct(static_cast<uint8_t>(val));
    ctx->acroCLAW.getYawPID()->setIntegralMaxLimPct(static_cast<uint8_t>(val));
    return ZP_ERROR_OK;
}
ZP_Error AMParamSetup::updateRollPitchLimitRate(AttitudeManager* ctx, float val) {
    if (val < 0.0f || val > 1080.0f) return ZP_ERROR_INVALID_ARG;
    const float RATE_RAD_PER_SEC = ZP_UNITS::deg2rad(val);
    ctx->acroCLAW.setRollLimitRate(RATE_RAD_PER_SEC);
    ctx->acroCLAW.setPitchLimitRate(RATE_RAD_PER_SEC);
    return ZP_ERROR_OK;
}
ZP_Error AMParamSetup::updateYawLimitRate(AttitudeManager* ctx, float val) {
    if (val < 0.0f || val > 1080.0f) return ZP_ERROR_INVALID_ARG;
    ctx->acroCLAW.setYawLimitRate(ZP_UNITS::deg2rad(val));
    return ZP_ERROR_OK;
}
// Stabilize callbacks
ZP_Error AMParamSetup::updateAngPIDRollKp(AttitudeManager* ctx, float val) {
    if (val < 0.0f) return ZP_ERROR_INVALID_ARG;
    ctx->stabilizeCLAW.getRollPID()->setKp(val);
    return ZP_ERROR_OK;
}
ZP_Error AMParamSetup::updateAngPIDRollKi(AttitudeManager* ctx, float val) {
    if (val < 0.0f) return ZP_ERROR_INVALID_ARG;
    ctx->stabilizeCLAW.getRollPID()->setKi(val);
    return ZP_ERROR_OK;
}
ZP_Error AMParamSetup::updateAngPIDRollKd(AttitudeManager* ctx, float val) {
    if (val < 0.0f) return ZP_ERROR_INVALID_ARG;
    ctx->stabilizeCLAW.getRollPID()->setKd(val);
    return ZP_ERROR_OK;
}
ZP_Error AMParamSetup::updateAngPIDRollTau(AttitudeManager* ctx, float val) {
    if (val < 0.0f) return ZP_ERROR_INVALID_ARG;
    ctx->stabilizeCLAW.getRollPID()->setTau(val);
    return ZP_ERROR_OK;
}
ZP_Error AMParamSetup::updateAngPIDRollIMax(AttitudeManager* ctx, float val) {
    if (val < 0.0f || val > 100.0f) return ZP_ERROR_INVALID_ARG;
    ctx->stabilizeCLAW.getRollPID()->setIntegralMinLimPct(static_cast<uint8_t>(val));
    ctx->stabilizeCLAW.getRollPID()->setIntegralMaxLimPct(static_cast<uint8_t>(val));
    return ZP_ERROR_OK;
}
ZP_Error AMParamSetup::updateAngPIDPitchKp(AttitudeManager* ctx, float val) {
    if (val < 0.0f) return ZP_ERROR_INVALID_ARG;
    ctx->stabilizeCLAW.getPitchPID()->setKp(val);
    return ZP_ERROR_OK;
}
ZP_Error AMParamSetup::updateAngPIDPitchKi(AttitudeManager* ctx, float val) {
    if (val < 0.0f) return ZP_ERROR_INVALID_ARG;
    ctx->stabilizeCLAW.getPitchPID()->setKi(val);
    return ZP_ERROR_OK;
}
ZP_Error AMParamSetup::updateAngPIDPitchKd(AttitudeManager* ctx, float val) {
    if (val < 0.0f) return ZP_ERROR_INVALID_ARG;
    ctx->stabilizeCLAW.getPitchPID()->setKd(val);
    return ZP_ERROR_OK;
}
ZP_Error AMParamSetup::updateAngPIDPitchTau(AttitudeManager* ctx, float val) {
    if (val < 0.0f) return ZP_ERROR_INVALID_ARG;
    ctx->stabilizeCLAW.getPitchPID()->setTau(val);
    return ZP_ERROR_OK;
}
ZP_Error AMParamSetup::updateAngPIDPitchIMax(AttitudeManager* ctx, float val) {
    if (val < 0.0f || val > 100.0f) return ZP_ERROR_INVALID_ARG;
    ctx->stabilizeCLAW.getPitchPID()->setIntegralMinLimPct(static_cast<uint8_t>(val));
    ctx->stabilizeCLAW.getPitchPID()->setIntegralMaxLimPct(static_cast<uint8_t>(val));
    return ZP_ERROR_OK;
}
ZP_Error AMParamSetup::updateRollPitchLimitAng(AttitudeManager* ctx, float val) {
    if (val < 0.0f || val > 45.0f) return ZP_ERROR_INVALID_ARG;
    ctx->stabilizeCLAW.setRollPitchLimitAngle(val);
    return ZP_ERROR_OK;
}
static constexpr float MOT_SPIN_RANGE_MIN_SEPARATION = 0.05f; // Guard against motSpinMin == motSpinMax, which would give no RPY authority

ZP_Error AMParamSetup::updateMotSpinMin(AttitudeManager* ctx, float val) {
    if (val < 0.0f || val > 1.0f || val > ctx->motSpinMax - MOT_SPIN_RANGE_MIN_SEPARATION) return ZP_ERROR_INVALID_ARG;
    ctx->motSpinMin = val;
    return ZP_ERROR_OK;
}
ZP_Error AMParamSetup::updateMotSpinMax(AttitudeManager* ctx, float val) {
    if (val < 0.0f || val > 1.0f || val < ctx->motSpinMin + MOT_SPIN_RANGE_MIN_SEPARATION) return ZP_ERROR_INVALID_ARG;
    ctx->motSpinMax = val;
    return ZP_ERROR_OK;
}
ZP_Error AMParamSetup::updateMotSpinArm(AttitudeManager* ctx, float val) {
    if (val < 0.0f || val > 1.0f) return ZP_ERROR_INVALID_ARG;
    ctx->motSpinArm = val;
    return ZP_ERROR_OK;
}
#endif

// FFT Harmonic Notch Filter callbacks (only do bound checking as they cannot change at runtime)
ZP_Error AMParamSetup::updateHarmonicNotchEnabled(AttitudeManager* ctx, float val) {
    // Must be 0 or 1
    int v = static_cast<int>(val);
    if (v != 0 && v != 1) return ZP_ERROR_INVALID_ARG;
    return ZP_ERROR_OK;
}

ZP_Error AMParamSetup::updateHarmonicNotchWindowSize(AttitudeManager* ctx, float val) {
    // Must be power of 2 between 32 and 1024
    int v = static_cast<int>(val);
    if (v < 32 || v > 1024 || (v & (v - 1)) != 0) return ZP_ERROR_INVALID_ARG;
    return ZP_ERROR_OK;
}

ZP_Error AMParamSetup::updateHarmonicNotchMinFreqHz(AttitudeManager* ctx, float val) {
    // Must be between 20 and 400 Hz
    if (val < 20.0f || val > 400.0f) return ZP_ERROR_INVALID_ARG;
    return ZP_ERROR_OK;
}

ZP_Error AMParamSetup::updateHarmonicNotchBandwidthHz(AttitudeManager* ctx, float val) {
    // Must be between 5 and 250 Hz
    if (val < 5.0f || val > 250.0f) return ZP_ERROR_INVALID_ARG;
    return ZP_ERROR_OK;
}

ZP_Error AMParamSetup::updateHarmonicNotchAttenuationDB(AttitudeManager* ctx, float val) {
    // Must be between 5 and 50 dB
    if (val < 5.0f || val > 50.0f) return ZP_ERROR_INVALID_ARG;
    return ZP_ERROR_OK;
}

ZP_Error AMParamSetup::updateHarmonicNotchHarmonicsMask(AttitudeManager* ctx, float val) {
    // Must be between 0 and 0xFFFF
    int v = static_cast<int>(val);
    if (v < 0 || v > 0xFFFF) return ZP_ERROR_INVALID_ARG;
    return ZP_ERROR_OK;
}

// Servo field helpers
ZP_Error AMParamSetup::setServoTrim(AttitudeManager* ctx, uint8_t ch, float val) {
    if (ch >= ctx->mainMotorGroup->motorCount || val < 0.0f || val > 2000.0f) return ZP_ERROR_INVALID_ARG;
    ctx->mainMotorGroup->motors[ch].trim = usToPercent(val);
    return ZP_ERROR_OK;
}
ZP_Error AMParamSetup::setServoMin(AttitudeManager* ctx, uint8_t ch, float val) {
    if (ch >= ctx->mainMotorGroup->motorCount || val < 0.0f || val > 2000.0f) return ZP_ERROR_INVALID_ARG;
    ctx->mainMotorGroup->motors[ch].min = usToPercent(val);
    return ZP_ERROR_OK;
}
ZP_Error AMParamSetup::setServoMax(AttitudeManager* ctx, uint8_t ch, float val) {
    if (ch >= ctx->mainMotorGroup->motorCount || val < 0.0f || val > 2000.0f) return ZP_ERROR_INVALID_ARG;
    ctx->mainMotorGroup->motors[ch].max = usToPercent(val);
    return ZP_ERROR_OK;
}
ZP_Error AMParamSetup::setServoReversed(AttitudeManager* ctx, uint8_t ch, float val) {
    int v = static_cast<int>(val);
    if (ch >= ctx->mainMotorGroup->motorCount || (v != 0 && v != 1)) return ZP_ERROR_INVALID_ARG;
    ctx->mainMotorGroup->motors[ch].isInverted = v != 0;
    return ZP_ERROR_OK;
}
ZP_Error AMParamSetup::setServoFunction(AttitudeManager* ctx, uint8_t ch, float val) {
    if (ch >= ctx->mainMotorGroup->motorCount) return ZP_ERROR_INVALID_ARG;
    ctx->mainMotorGroup->motors[ch].function = static_cast<MotorFunction_e>(static_cast<int16_t>(val));
    return ZP_ERROR_OK;
}
