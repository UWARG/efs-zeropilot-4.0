#include "am_param_setup.hpp"
#include "attitude_manager.hpp"
#include "zp_params.hpp"
#include "motor_functions.hpp"
#include "unit_conversions.hpp"

static inline int usToPercent(float us) {
    return static_cast<int>((us - 1000.0f) / 10.0f);
}

AMParamSetup::AMParamSetup(AttitudeManager* am) : am(am) {}

void AMParamSetup::loadAllParams() {
    #ifdef PLANE
    // FBWA params
    am->fbwaCLAW.setRollPIDConstants(
        ZP_PARAM::get(ZP_PARAM_ID::RLL2SRV_P),
        ZP_PARAM::get(ZP_PARAM_ID::RLL2SRV_I),
        ZP_PARAM::get(ZP_PARAM_ID::RLL2SRV_D),
        ZP_PARAM::get(ZP_PARAM_ID::RLL2SRV_TAU),
        ZP_PARAM::get(ZP_PARAM_ID::RLL2SRV_IMAX)
    );
    am->fbwaCLAW.setPitchPIDConstants(
        ZP_PARAM::get(ZP_PARAM_ID::PTCH2SRV_P),
        ZP_PARAM::get(ZP_PARAM_ID::PTCH2SRV_I),
        ZP_PARAM::get(ZP_PARAM_ID::PTCH2SRV_D),
        ZP_PARAM::get(ZP_PARAM_ID::PTCH2SRV_TAU),
        ZP_PARAM::get(ZP_PARAM_ID::PTCH2SRV_IMAX)
    );
    am->fbwaCLAW.setYawRudderMixingConstant(ZP_PARAM::get(ZP_PARAM_ID::KFF_RDDRMIX));
    am->fbwaCLAW.setRollLimitDeg(ZP_PARAM::get(ZP_PARAM_ID::ROLL_LIMIT_DEG));
    am->fbwaCLAW.setPitchLimitMaxDeg(ZP_PARAM::get(ZP_PARAM_ID::PTCH_LIM_MAX_DEG));
    am->fbwaCLAW.setPitchLimitMinDeg(ZP_PARAM::get(ZP_PARAM_ID::PTCH_LIM_MIN_DEG));
    #endif
    #ifdef QUADCOPTER
    // ACRO params
    am->acroCLAW.setRollPIDConstants(
        ZP_PARAM::get(ZP_PARAM_ID::ATC_RAT_RLL_P),
        ZP_PARAM::get(ZP_PARAM_ID::ATC_RAT_RLL_I),
        ZP_PARAM::get(ZP_PARAM_ID::ATC_RAT_RLL_D),
        ZP_PARAM::get(ZP_PARAM_ID::ATC_RAT_RLL_TAU),
        ZP_PARAM::get(ZP_PARAM_ID::ATC_RAT_RLL_IMAX)
    );
    am->acroCLAW.setPitchPIDConstants(
        ZP_PARAM::get(ZP_PARAM_ID::ATC_RAT_PIT_P),
        ZP_PARAM::get(ZP_PARAM_ID::ATC_RAT_PIT_I),
        ZP_PARAM::get(ZP_PARAM_ID::ATC_RAT_PIT_D),
        ZP_PARAM::get(ZP_PARAM_ID::ATC_RAT_PIT_TAU),
        ZP_PARAM::get(ZP_PARAM_ID::ATC_RAT_PIT_IMAX)
    );
    am->acroCLAW.setYawPIDConstants(
        ZP_PARAM::get(ZP_PARAM_ID::ATC_RAT_YAW_P),
        ZP_PARAM::get(ZP_PARAM_ID::ATC_RAT_YAW_I),
        ZP_PARAM::get(ZP_PARAM_ID::ATC_RAT_YAW_D),
        ZP_PARAM::get(ZP_PARAM_ID::ATC_RAT_YAW_TAU),
        ZP_PARAM::get(ZP_PARAM_ID::ATC_RAT_YAW_IMAX)
    );
    am->acroCLAW.setRollLimitRate(ZP_UNITS::deg2rad(ZP_PARAM::get(ZP_PARAM_ID::ACRO_RP_RATE)));
    am->acroCLAW.setPitchLimitRate(ZP_UNITS::deg2rad(ZP_PARAM::get(ZP_PARAM_ID::ACRO_RP_RATE)));
    am->acroCLAW.setYawLimitRate(ZP_UNITS::deg2rad(ZP_PARAM::get(ZP_PARAM_ID::ACRO_Y_RATE)));
    #endif

    // Servo params
    auto loadMotor = [&](uint8_t ch, ZP_PARAM_ID trim, ZP_PARAM_ID min, ZP_PARAM_ID max, ZP_PARAM_ID rev, ZP_PARAM_ID func) {
        if (ch >= am->mainMotorGroup->motorCount) return;
        MotorInstance_t* m = &am->mainMotorGroup->motors[ch];
        m->trim       = usToPercent(ZP_PARAM::get(trim));
        m->min        = usToPercent(ZP_PARAM::get(min));
        m->max        = usToPercent(ZP_PARAM::get(max));
        m->isInverted = static_cast<int>(ZP_PARAM::get(rev)) != 0;
        m->function   = static_cast<MotorFunction_e>(static_cast<int16_t>(ZP_PARAM::get(func)));
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
}

// Macro to bind all 5 fields for a single servo channel
#define AM_PARAM_SETUP_BIND_SERVO_CB(N) \
    ZP_PARAM::bindCallback(ZP_PARAM_ID::SERVO##N##_TRIM,     am, cbServoTrim<N-1>);     \
    ZP_PARAM::bindCallback(ZP_PARAM_ID::SERVO##N##_MIN,      am, cbServoMin<N-1>);      \
    ZP_PARAM::bindCallback(ZP_PARAM_ID::SERVO##N##_MAX,      am, cbServoMax<N-1>);      \
    ZP_PARAM::bindCallback(ZP_PARAM_ID::SERVO##N##_REVERSED, am, cbServoReversed<N-1>); \
    ZP_PARAM::bindCallback(ZP_PARAM_ID::SERVO##N##_FUNCTION, am, cbServoFunction<N-1>);

void AMParamSetup::bindAllParamCallbacks() {
    // FBWA
    #ifdef PLANE
    ZP_PARAM::bindCallback(ZP_PARAM_ID::RLL2SRV_P,        am, updatePIDRollKp);
    ZP_PARAM::bindCallback(ZP_PARAM_ID::RLL2SRV_I,        am, updatePIDRollKi);
    ZP_PARAM::bindCallback(ZP_PARAM_ID::RLL2SRV_D,        am, updatePIDRollKd);
    ZP_PARAM::bindCallback(ZP_PARAM_ID::RLL2SRV_TAU,      am, updatePIDRollTau);
    ZP_PARAM::bindCallback(ZP_PARAM_ID::RLL2SRV_IMAX,     am, updatePIDRollIMax);
    ZP_PARAM::bindCallback(ZP_PARAM_ID::PTCH2SRV_P,       am, updatePIDPitchKp);
    ZP_PARAM::bindCallback(ZP_PARAM_ID::PTCH2SRV_I,       am, updatePIDPitchKi);
    ZP_PARAM::bindCallback(ZP_PARAM_ID::PTCH2SRV_D,       am, updatePIDPitchKd);
    ZP_PARAM::bindCallback(ZP_PARAM_ID::PTCH2SRV_TAU,     am, updatePIDPitchTau);
    ZP_PARAM::bindCallback(ZP_PARAM_ID::PTCH2SRV_IMAX,    am, updatePIDPitchIMax);
    ZP_PARAM::bindCallback(ZP_PARAM_ID::KFF_RDDRMIX,      am, updateKffRddrmix);
    ZP_PARAM::bindCallback(ZP_PARAM_ID::ROLL_LIMIT_DEG,   am, updateRollLimitDeg);
    ZP_PARAM::bindCallback(ZP_PARAM_ID::PTCH_LIM_MAX_DEG, am, updatePitchLimMaxDeg);
    ZP_PARAM::bindCallback(ZP_PARAM_ID::PTCH_LIM_MIN_DEG, am, updatePitchLimMinDeg);
    #endif
    #ifdef QUADCOPTER
    ZP_PARAM::bindCallback(ZP_PARAM_ID::ATC_RAT_RLL_P,        am, updatePIDRollKp);
    ZP_PARAM::bindCallback(ZP_PARAM_ID::ATC_RAT_RLL_I,        am, updatePIDRollKi);
    ZP_PARAM::bindCallback(ZP_PARAM_ID::ATC_RAT_RLL_D,        am, updatePIDRollKd);
    ZP_PARAM::bindCallback(ZP_PARAM_ID::ATC_RAT_RLL_TAU,      am, updatePIDRollTau);
    ZP_PARAM::bindCallback(ZP_PARAM_ID::ATC_RAT_RLL_IMAX,     am, updatePIDRollIMax);
    ZP_PARAM::bindCallback(ZP_PARAM_ID::ATC_RAT_PIT_P,        am, updatePIDPitchKp);
    ZP_PARAM::bindCallback(ZP_PARAM_ID::ATC_RAT_PIT_I,        am, updatePIDPitchKi);
    ZP_PARAM::bindCallback(ZP_PARAM_ID::ATC_RAT_PIT_D,        am, updatePIDPitchKd);
    ZP_PARAM::bindCallback(ZP_PARAM_ID::ATC_RAT_PIT_TAU,      am, updatePIDPitchTau);
    ZP_PARAM::bindCallback(ZP_PARAM_ID::ATC_RAT_PIT_IMAX,     am, updatePIDPitchIMax);
    ZP_PARAM::bindCallback(ZP_PARAM_ID::ATC_RAT_YAW_P,        am, updatePIDYawKp);
    ZP_PARAM::bindCallback(ZP_PARAM_ID::ATC_RAT_YAW_I,        am, updatePIDYawKi);
    ZP_PARAM::bindCallback(ZP_PARAM_ID::ATC_RAT_YAW_D,        am, updatePIDYawKd);
    ZP_PARAM::bindCallback(ZP_PARAM_ID::ATC_RAT_YAW_TAU,      am, updatePIDYawTau);
    ZP_PARAM::bindCallback(ZP_PARAM_ID::ATC_RAT_YAW_IMAX,     am, updatePIDYawIMax);
    ZP_PARAM::bindCallback(ZP_PARAM_ID::ACRO_RP_RATE,         am, updateRollPitchLimitRate);
    ZP_PARAM::bindCallback(ZP_PARAM_ID::ACRO_Y_RATE,          am, updateYawLimitRate);
    #endif

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
}

#undef AM_PARAM_SETUP_BIND_SERVO_CB

// FBWA callbacks

bool AMParamSetup::updatePIDRollKp(AttitudeManager* ctx, float val) {
    if (val < 0.0f) return false;
    #ifdef PLANE
    ctx->fbwaCLAW.getRollPID()->setKp(val);
    #endif
    #ifdef QUADCOPTER
    ctx->acroCLAW.getRollPID()->setKp(val);
    #endif
    return true;
}
bool AMParamSetup::updatePIDRollKi(AttitudeManager* ctx, float val) {
    if (val < 0.0f) return false;
    #ifdef PLANE
    ctx->fbwaCLAW.getRollPID()->setKi(val);
    #endif
    #ifdef QUADCOPTER
    ctx->acroCLAW.getRollPID()->setKi(val);
    #endif
    return true;
}
bool AMParamSetup::updatePIDRollKd(AttitudeManager* ctx, float val) {
    if (val < 0.0f) return false;
    #ifdef PLANE
    ctx->fbwaCLAW.getRollPID()->setKd(val);
    #endif
    #ifdef QUADCOPTER
    ctx->acroCLAW.getRollPID()->setKd(val);
    #endif
    return true;
}
bool AMParamSetup::updatePIDRollTau(AttitudeManager* ctx, float val) {
    if (val < 0.0f) return false;
    #ifdef PLANE
    ctx->fbwaCLAW.getRollPID()->setTau(val);
    #endif
    #ifdef QUADCOPTER
    ctx->acroCLAW.getRollPID()->setTau(val);
    #endif
    return true;
}
bool AMParamSetup::updatePIDRollIMax(AttitudeManager* ctx, float val) {
    if (val < 0.0f || val > 100.0f) return false;
    #ifdef PLANE
    ctx->fbwaCLAW.getRollPID()->setIntegralMinLimPct(static_cast<uint8_t>(val));
    ctx->fbwaCLAW.getRollPID()->setIntegralMaxLimPct(static_cast<uint8_t>(val));
    #endif
    #ifdef QUADCOPTER
    ctx->acroCLAW.getRollPID()->setIntegralMinLimPct(static_cast<uint8_t>(val));
    ctx->acroCLAW.getRollPID()->setIntegralMaxLimPct(static_cast<uint8_t>(val));
    #endif
    return true;
}
bool AMParamSetup::updatePIDPitchKp(AttitudeManager* ctx, float val) {
    if (val < 0.0f) return false;
    #ifdef PLANE
    ctx->fbwaCLAW.getPitchPID()->setKp(val);
    #endif
    #ifdef QUADCOPTER
    ctx->acroCLAW.getPitchPID()->setKp(val);
    #endif
    return true;
}
bool AMParamSetup::updatePIDPitchKi(AttitudeManager* ctx, float val) {
    if (val < 0.0f) return false;
    #ifdef PLANE
    ctx->fbwaCLAW.getPitchPID()->setKi(val);
    #endif
    #ifdef QUADCOPTER
    ctx->acroCLAW.getPitchPID()->setKi(val);
    #endif
    return true;
}
bool AMParamSetup::updatePIDPitchKd(AttitudeManager* ctx, float val) {
    if (val < 0.0f) return false;
    #ifdef PLANE
    ctx->fbwaCLAW.getPitchPID()->setKd(val);
    #endif
    #ifdef QUADCOPTER
    ctx->acroCLAW.getPitchPID()->setKd(val);
    #endif
    return true;
}
bool AMParamSetup::updatePIDPitchTau(AttitudeManager* ctx, float val) {
    if (val < 0.0f) return false;
    #ifdef PLANE
    ctx->fbwaCLAW.getPitchPID()->setTau(val);
    #endif
    #ifdef QUADCOPTER
    ctx->acroCLAW.getPitchPID()->setTau(val);
    #endif
    return true;
}
bool AMParamSetup::updatePIDPitchIMax(AttitudeManager* ctx, float val) {
    if (val < 0.0f || val > 100.0f) return false;
    #ifdef PLANE
    ctx->fbwaCLAW.getPitchPID()->setIntegralMinLimPct(static_cast<uint8_t>(val));
    ctx->fbwaCLAW.getPitchPID()->setIntegralMaxLimPct(static_cast<uint8_t>(val));
    return true;
    #endif
    #ifdef QUADCOPTER
    ctx->acroCLAW.getPitchPID()->setIntegralMinLimPct(static_cast<uint8_t>(val));
    ctx->acroCLAW.getPitchPID()->setIntegralMaxLimPct(static_cast<uint8_t>(val));
    return true;
    #endif
}
#ifdef PLANE
bool AMParamSetup::updateKffRddrmix(AttitudeManager* ctx, float val) {
    if (val < 0.0f || val > 1.0f) return false;
    ctx->fbwaCLAW.setYawRudderMixingConstant(val);
    return true;
}
bool AMParamSetup::updateRollLimitDeg(AttitudeManager* ctx, float val) {
    if (val < 0.0f || val > 90.0f) return false;
    ctx->fbwaCLAW.setRollLimitDeg(val);
    return true;
}
bool AMParamSetup::updatePitchLimMaxDeg(AttitudeManager* ctx, float val) {
    if (val < 0.0f || val > 90.0f) return false;
    ctx->fbwaCLAW.setPitchLimitMaxDeg(val);
    return true;
}
bool AMParamSetup::updatePitchLimMinDeg(AttitudeManager* ctx, float val) {
    if (val < -90.0f || val > 0.0f) return false;
    ctx->fbwaCLAW.setPitchLimitMinDeg(val);
    return true;
}
#endif
#ifdef QUADCOPTER
bool AMParamSetup::updatePIDYawKp(AttitudeManager* ctx, float val) {
    if (val < 0.0f) return false;
    ctx->acroCLAW.getYawPID()->setKp(val);
    return true;
}
bool AMParamSetup::updatePIDYawKi(AttitudeManager* ctx, float val) {
    if (val < 0.0f) return false;
    ctx->acroCLAW.getYawPID()->setKi(val);
    return true;
}
bool AMParamSetup::updatePIDYawKd(AttitudeManager* ctx, float val) {
    if (val < 0.0f) return false;
    ctx->acroCLAW.getYawPID()->setKd(val);
    return true;
}
bool AMParamSetup::updatePIDYawTau(AttitudeManager* ctx, float val) {
    if (val < 0.0f) return false;
    ctx->acroCLAW.getYawPID()->setTau(val);
    return true;
}
bool AMParamSetup::updatePIDYawIMax(AttitudeManager* ctx, float val) {
    if (val < 0.0f || val > 100.0f) return false;
    ctx->acroCLAW.getYawPID()->setIntegralMinLimPct(static_cast<uint8_t>(val));
    ctx->acroCLAW.getYawPID()->setIntegralMaxLimPct(static_cast<uint8_t>(val));
    return true;
}
bool AMParamSetup::updateRollPitchLimitRate(AttitudeManager* ctx, float val) {
    if (val < 0.0f || val > 1080.0f) return false;
    const float rateRadPerSec = ZP_UNITS::deg2rad(val);
    ctx->acroCLAW.setRollLimitRate(rateRadPerSec);
    ctx->acroCLAW.setPitchLimitRate(rateRadPerSec);
    return true;
}
bool AMParamSetup::updateYawLimitRate(AttitudeManager* ctx, float val) {
    if (val < 0.0f || val > 1080.0f) return false;
    ctx->acroCLAW.setYawLimitRate(ZP_UNITS::deg2rad(val));
    return true;
}
#endif

// Servo field helpers

bool AMParamSetup::setServoTrim(AttitudeManager* ctx, uint8_t ch, float val) {
    if (ch >= ctx->mainMotorGroup->motorCount || val < 0.0f || val > 2000.0f) return false;
    ctx->mainMotorGroup->motors[ch].trim = usToPercent(val);
    return true;
}
bool AMParamSetup::setServoMin(AttitudeManager* ctx, uint8_t ch, float val) {
    if (ch >= ctx->mainMotorGroup->motorCount || val < 0.0f || val > 2000.0f) return false;
    ctx->mainMotorGroup->motors[ch].min = usToPercent(val);
    return true;
}
bool AMParamSetup::setServoMax(AttitudeManager* ctx, uint8_t ch, float val) {
    if (ch >= ctx->mainMotorGroup->motorCount || val < 0.0f || val > 2000.0f) return false;
    ctx->mainMotorGroup->motors[ch].max = usToPercent(val);
    return true;
}
bool AMParamSetup::setServoReversed(AttitudeManager* ctx, uint8_t ch, float val) {
    int v = static_cast<int>(val);
    if (ch >= ctx->mainMotorGroup->motorCount || (v != 0 && v != 1)) return false;
    ctx->mainMotorGroup->motors[ch].isInverted = v != 0;
    return true;
}
bool AMParamSetup::setServoFunction(AttitudeManager* ctx, uint8_t ch, float val) {
    if (ch >= ctx->mainMotorGroup->motorCount) return false;
    ctx->mainMotorGroup->motors[ch].function = static_cast<MotorFunction_e>(static_cast<int16_t>(val));
    return true;
}
