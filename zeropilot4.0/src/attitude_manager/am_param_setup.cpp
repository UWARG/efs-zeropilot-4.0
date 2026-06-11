#include "am_param_setup.hpp"
#include "attitude_manager.hpp"
#include "zp_params.hpp"
#include "motor_functions.hpp"

static inline int usToPercent(float us) {
    return static_cast<int>((us - 1000.0f) / 10.0f);
}

AMParamSetup::AMParamSetup(AttitudeManager* am) : am(am) {}

void AMParamSetup::loadAllParams() {
    #ifdef FIXED_WING
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
    am->acroCLAW.setRollPIDConstants(
        ZP_PARAM::get(ZP_PARAM_ID::ACT_RATE_RLL_P),
        ZP_PARAM::get(ZP_PARAM_ID::ACT_RATE_RLL_I),
        ZP_PARAM::get(ZP_PARAM_ID::ACT_RATE_RLL_D),
        ZP_PARAM::get(ZP_PARAM_ID::ACT_RATE_RLL_TAU),
        ZP_PARAM::get(ZP_PARAM_ID::ACT_RATE_RLL_IMAX)
    );
    am->acroCLAW.setPitchPIDConstants(
        ZP_PARAM::get(ZP_PARAM_ID::ACT_RATE_PTCH_P),
        ZP_PARAM::get(ZP_PARAM_ID::ACT_RATE_PTCH_I),
        ZP_PARAM::get(ZP_PARAM_ID::ACT_RATE_PTCH_D),
        ZP_PARAM::get(ZP_PARAM_ID::ACT_RATE_PTCH_TAU),
        ZP_PARAM::get(ZP_PARAM_ID::ACT_RATE_PTCH_IMAX)
    );
    am->acroCLAW.setYawPIDConstants(
        ZP_PARAM::get(ZP_PARAM_ID::ACT_RATE_YAW_P),
        ZP_PARAM::get(ZP_PARAM_ID::ACT_RATE_YAW_I),
        ZP_PARAM::get(ZP_PARAM_ID::ACT_RATE_YAW_D),
        ZP_PARAM::get(ZP_PARAM_ID::ACT_RATE_YAW_TAU),
        ZP_PARAM::get(ZP_PARAM_ID::ACT_RATE_YAW_IMAX)
    );
    am->acroCLAW.setRollLimitRate(ZP_PARAM::get(ZP_PARAM_ID::ROLL_LIMIT_RATE));
    am->acroCLAW.setPitchLimitRate(ZP_PARAM::get(ZP_PARAM_ID::PTCH_LIMIT_RATE));
    am->acroCLAW.setYawLimitRate(ZP_PARAM::get(ZP_PARAM_ID::YAW_LIMIT_RATE));

    am->stabilizeCLAW.setRollPIDConstants(
        ZP_PARAM::get(ZP_PARAM_ID::ACT_ANG_RLL_P),
        ZP_PARAM::get(ZP_PARAM_ID::ACT_ANG_RLL_I),
        ZP_PARAM::get(ZP_PARAM_ID::ACT_ANG_RLL_D),
        ZP_PARAM::get(ZP_PARAM_ID::ACT_ANG_RLL_TAU),
        ZP_PARAM::get(ZP_PARAM_ID::ACT_ANG_RLL_IMAX)
    );
    am->stabilizeCLAW.setPitchPIDConstants(
        ZP_PARAM::get(ZP_PARAM_ID::ACT_ANG_PTCH_P),
        ZP_PARAM::get(ZP_PARAM_ID::ACT_ANG_PTCH_I),
        ZP_PARAM::get(ZP_PARAM_ID::ACT_ANG_PTCH_D),
        ZP_PARAM::get(ZP_PARAM_ID::ACT_ANG_PTCH_TAU),
        ZP_PARAM::get(ZP_PARAM_ID::ACT_ANG_PTCH_IMAX)
    );
    am->stabilizeCLAW.setRollLimitAngle(ZP_PARAM::get(ZP_PARAM_ID::ROLL_LIMIT_ANG));
    am->stabilizeCLAW.setPitchLimitAngle(ZP_PARAM::get(ZP_PARAM_ID::PTCH_LIMIT_ANG));
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
    #ifdef FIXED_WING
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
    ZP_PARAM::bindCallback(ZP_PARAM_ID::ACT_RATE_RLL_P,        am, updateRatePIDRollKp);
    ZP_PARAM::bindCallback(ZP_PARAM_ID::ACT_RATE_RLL_I,        am, updateRatePIDRollKi);
    ZP_PARAM::bindCallback(ZP_PARAM_ID::ACT_RATE_RLL_D,        am, updateRatePIDRollKd);
    ZP_PARAM::bindCallback(ZP_PARAM_ID::ACT_RATE_RLL_TAU,      am, updateRatePIDRollTau);
    ZP_PARAM::bindCallback(ZP_PARAM_ID::ACT_RATE_RLL_IMAX,     am, updateRatePIDRollIMax);
    ZP_PARAM::bindCallback(ZP_PARAM_ID::ACT_RATE_PTCH_P,       am, updateRatePIDPitchKp);
    ZP_PARAM::bindCallback(ZP_PARAM_ID::ACT_RATE_PTCH_I,       am, updateRatePIDPitchKi);
    ZP_PARAM::bindCallback(ZP_PARAM_ID::ACT_RATE_PTCH_D,       am, updateRatePIDPitchKd);
    ZP_PARAM::bindCallback(ZP_PARAM_ID::ACT_RATE_PTCH_TAU,     am, updateRatePIDPitchTau);
    ZP_PARAM::bindCallback(ZP_PARAM_ID::ACT_RATE_PTCH_IMAX,    am, updateRatePIDPitchIMax);
    ZP_PARAM::bindCallback(ZP_PARAM_ID::ACT_RATE_YAW_P,        am, updateRatePIDYawKp);
    ZP_PARAM::bindCallback(ZP_PARAM_ID::ACT_RATE_YAW_I,        am, updateRatePIDYawKi);
    ZP_PARAM::bindCallback(ZP_PARAM_ID::ACT_RATE_YAW_D,        am, updateRatePIDYawKd);
    ZP_PARAM::bindCallback(ZP_PARAM_ID::ACT_RATE_YAW_TAU,      am, updateRatePIDYawTau);
    ZP_PARAM::bindCallback(ZP_PARAM_ID::ACT_RATE_YAW_IMAX,     am, updateRatePIDYawIMax);
    ZP_PARAM::bindCallback(ZP_PARAM_ID::ROLL_LIMIT_RATE,       am, updateRollLimitRate);
    ZP_PARAM::bindCallback(ZP_PARAM_ID::PTCH_LIMIT_RATE,       am, updatePitchLimitRate);
    ZP_PARAM::bindCallback(ZP_PARAM_ID::YAW_LIMIT_RATE,        am, updateYawLimitRate);

    ZP_PARAM::bindCallback(ZP_PARAM_ID::ACT_ANG_RLL_P,         am, updateAngPIDRollKp);
    ZP_PARAM::bindCallback(ZP_PARAM_ID::ACT_ANG_RLL_I,         am, updateAngPIDRollKi);
    ZP_PARAM::bindCallback(ZP_PARAM_ID::ACT_ANG_RLL_D,         am, updateAngPIDRollKd);
    ZP_PARAM::bindCallback(ZP_PARAM_ID::ACT_ANG_RLL_TAU,       am, updateAngPIDRollTau);
    ZP_PARAM::bindCallback(ZP_PARAM_ID::ACT_ANG_RLL_IMAX,      am, updateAngPIDRollIMax);
    ZP_PARAM::bindCallback(ZP_PARAM_ID::ACT_ANG_PTCH_P,        am, updateAngPIDPitchKp);
    ZP_PARAM::bindCallback(ZP_PARAM_ID::ACT_ANG_PTCH_I,        am, updateAngPIDPitchKi);
    ZP_PARAM::bindCallback(ZP_PARAM_ID::ACT_ANG_PTCH_D,        am, updateAngPIDPitchKd);
    ZP_PARAM::bindCallback(ZP_PARAM_ID::ACT_ANG_PTCH_TAU,      am, updateAngPIDPitchTau);
    ZP_PARAM::bindCallback(ZP_PARAM_ID::ACT_ANG_PTCH_IMAX,     am, updateAngPIDPitchIMax);
    ZP_PARAM::bindCallback(ZP_PARAM_ID::ROLL_LIMIT_ANG,       am, updateRollLimitAng);
    ZP_PARAM::bindCallback(ZP_PARAM_ID::PTCH_LIMIT_ANG,       am, updatePitchLimitAng);
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

#ifdef FIXED_WING
// FBWA callbacks
bool AMParamSetup::updatePIDRollKp(AttitudeManager* ctx, float val) {
    if (val < 0.0f) return false;
    ctx->fbwaCLAW.getRollPID()->setKp(val);
    return true;
}
bool AMParamSetup::updatePIDRollKi(AttitudeManager* ctx, float val) {
    if (val < 0.0f) return false;
    ctx->fbwaCLAW.getRollPID()->setKi(val);
    return true;
}
bool AMParamSetup::updatePIDRollKd(AttitudeManager* ctx, float val) {
    if (val < 0.0f) return false;
    ctx->fbwaCLAW.getRollPID()->setKd(val);
    return true;
}
bool AMParamSetup::updatePIDRollTau(AttitudeManager* ctx, float val) {
    if (val < 0.0f) return false;
    ctx->fbwaCLAW.getRollPID()->setTau(val);
    return true;
}
bool AMParamSetup::updatePIDRollIMax(AttitudeManager* ctx, float val) {
    if (val < 0.0f || val > 100.0f) return false;
    ctx->fbwaCLAW.getRollPID()->setIntegralMinLimPct(static_cast<uint8_t>(val));
    ctx->fbwaCLAW.getRollPID()->setIntegralMaxLimPct(static_cast<uint8_t>(val));
    return true;
}
bool AMParamSetup::updatePIDPitchKp(AttitudeManager* ctx, float val) {
    if (val < 0.0f) return false;
    ctx->fbwaCLAW.getPitchPID()->setKp(val);
    return true;
}
bool AMParamSetup::updatePIDPitchKi(AttitudeManager* ctx, float val) {
    if (val < 0.0f) return false;
    ctx->fbwaCLAW.getPitchPID()->setKi(val);
    return true;
}
bool AMParamSetup::updatePIDPitchKd(AttitudeManager* ctx, float val) {
    if (val < 0.0f) return false;
    ctx->fbwaCLAW.getPitchPID()->setKd(val);
    return true;
}
bool AMParamSetup::updatePIDPitchTau(AttitudeManager* ctx, float val) {
    if (val < 0.0f) return false;
    ctx->fbwaCLAW.getPitchPID()->setTau(val);
    return true;
}
bool AMParamSetup::updatePIDPitchIMax(AttitudeManager* ctx, float val) {
    if (val < 0.0f || val > 100.0f) return false;
    ctx->fbwaCLAW.getPitchPID()->setIntegralMinLimPct(static_cast<uint8_t>(val));
    ctx->fbwaCLAW.getPitchPID()->setIntegralMaxLimPct(static_cast<uint8_t>(val));
    return true;
}
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
// Acro callbacks
bool AMParamSetup::updateRatePIDRollKp(AttitudeManager* ctx, float val) {
    if (val < 0.0f) return false;
    ctx->acroCLAW.getRollPID()->setKp(val);
    return true;
}
bool AMParamSetup::updateRatePIDRollKi(AttitudeManager* ctx, float val) {
    if (val < 0.0f) return false;
    ctx->acroCLAW.getRollPID()->setKi(val);
    return true;
}
bool AMParamSetup::updateRatePIDRollKd(AttitudeManager* ctx, float val) {
    if (val < 0.0f) return false;
    ctx->acroCLAW.getRollPID()->setKd(val);
    return true;
}
bool AMParamSetup::updateRatePIDRollTau(AttitudeManager* ctx, float val) {
    if (val < 0.0f) return false;
    ctx->acroCLAW.getRollPID()->setTau(val);
    return true;
}
bool AMParamSetup::updateRatePIDRollIMax(AttitudeManager* ctx, float val) {
    if (val < 0.0f || val > 100.0f) return false;
    ctx->acroCLAW.getRollPID()->setIntegralMinLimPct(static_cast<uint8_t>(val));
    ctx->acroCLAW.getRollPID()->setIntegralMaxLimPct(static_cast<uint8_t>(val));
    return true;
}
bool AMParamSetup::updateRatePIDPitchKp(AttitudeManager* ctx, float val) {
    if (val < 0.0f) return false;
    ctx->acroCLAW.getPitchPID()->setKp(val);
    return true;
}
bool AMParamSetup::updateRatePIDPitchKi(AttitudeManager* ctx, float val) {
    if (val < 0.0f) return false;
    ctx->acroCLAW.getPitchPID()->setKi(val);
    return true;
}
bool AMParamSetup::updateRatePIDPitchKd(AttitudeManager* ctx, float val) {
    if (val < 0.0f) return false;
    ctx->acroCLAW.getPitchPID()->setKd(val);
    return true;
}
bool AMParamSetup::updateRatePIDPitchTau(AttitudeManager* ctx, float val) {
    if (val < 0.0f) return false;
    ctx->acroCLAW.getPitchPID()->setTau(val);
    return true;
}
bool AMParamSetup::updateRatePIDPitchIMax(AttitudeManager* ctx, float val) {
    if (val < 0.0f || val > 100.0f) return false;
    ctx->acroCLAW.getPitchPID()->setIntegralMinLimPct(static_cast<uint8_t>(val));
    ctx->acroCLAW.getPitchPID()->setIntegralMaxLimPct(static_cast<uint8_t>(val));
    return true;
}
bool AMParamSetup::updateRatePIDYawKp(AttitudeManager* ctx, float val) {
    if (val < 0.0f) return false;
    ctx->acroCLAW.getYawPID()->setKp(val);
    return true;
}
bool AMParamSetup::updateRatePIDYawKi(AttitudeManager* ctx, float val) {
    if (val < 0.0f) return false;
    ctx->acroCLAW.getYawPID()->setKi(val);
    return true;
}
bool AMParamSetup::updateRatePIDYawKd(AttitudeManager* ctx, float val) {
    if (val < 0.0f) return false;
    ctx->acroCLAW.getYawPID()->setKd(val);
    return true;
}
bool AMParamSetup::updateRatePIDYawTau(AttitudeManager* ctx, float val) {
    if (val < 0.0f) return false;
    ctx->acroCLAW.getYawPID()->setTau(val);
    return true;
}
bool AMParamSetup::updateRatePIDYawIMax(AttitudeManager* ctx, float val) {
    if (val < 0.0f || val > 100.0f) return false;
    ctx->acroCLAW.getYawPID()->setIntegralMinLimPct(static_cast<uint8_t>(val));
    ctx->acroCLAW.getYawPID()->setIntegralMaxLimPct(static_cast<uint8_t>(val));
    return true;
}
bool AMParamSetup::updateRollLimitRate(AttitudeManager* ctx, float val) {
    if (val < 0.0f || val > 6.28318f) return false;
    ctx->acroCLAW.setRollLimitRate(val);
    return true;
}
bool AMParamSetup::updatePitchLimitRate(AttitudeManager* ctx, float val) {
    if (val < 0.0f || val > 6.28318f) return false;
    ctx->acroCLAW.setPitchLimitRate(val);
    return true;
}
bool AMParamSetup::updateYawLimitRate(AttitudeManager* ctx, float val) {
    if (val < 0.0f || val > 6.28318f) return false;
    ctx->acroCLAW.setYawLimitRate(val);
    return true;
}
// Stabilize callbacks
bool AMParamSetup::updateAngPIDRollKp(AttitudeManager* ctx, float val) {
    if (val < 0.0f) return false;
    ctx->stabilizeCLAW.getRollPID()->setKp(val);
    return true;
}
bool AMParamSetup::updateAngPIDRollKi(AttitudeManager* ctx, float val) {
    if (val < 0.0f) return false;
    ctx->stabilizeCLAW.getRollPID()->setKi(val);
    return true;
}
bool AMParamSetup::updateAngPIDRollKd(AttitudeManager* ctx, float val) {
    if (val < 0.0f) return false;
    ctx->stabilizeCLAW.getRollPID()->setKd(val);
    return true;
}
bool AMParamSetup::updateAngPIDRollTau(AttitudeManager* ctx, float val) {
    if (val < 0.0f) return false;
    ctx->stabilizeCLAW.getRollPID()->setTau(val);
    return true;
}
bool AMParamSetup::updateAngPIDRollIMax(AttitudeManager* ctx, float val) {
    if (val < 0.0f || val > 100.0f) return false;
    ctx->stabilizeCLAW.getRollPID()->setIntegralMinLimPct(static_cast<uint8_t>(val));
    ctx->stabilizeCLAW.getRollPID()->setIntegralMaxLimPct(static_cast<uint8_t>(val));
    return true;
}
bool AMParamSetup::updateAngPIDPitchKp(AttitudeManager* ctx, float val) {
    if (val < 0.0f) return false;
    ctx->stabilizeCLAW.getPitchPID()->setKp(val);
    return true;
}
bool AMParamSetup::updateAngPIDPitchKi(AttitudeManager* ctx, float val) {
    if (val < 0.0f) return false;
    ctx->stabilizeCLAW.getPitchPID()->setKi(val);
    return true;
}
bool AMParamSetup::updateAngPIDPitchKd(AttitudeManager* ctx, float val) {
    if (val < 0.0f) return false;
    ctx->stabilizeCLAW.getPitchPID()->setKd(val);
    return true;
}
bool AMParamSetup::updateAngPIDPitchTau(AttitudeManager* ctx, float val) {
    if (val < 0.0f) return false;
    ctx->stabilizeCLAW.getPitchPID()->setTau(val);
    return true;
}
bool AMParamSetup::updateAngPIDPitchIMax(AttitudeManager* ctx, float val) {
    if (val < 0.0f || val > 100.0f) return false;
    ctx->stabilizeCLAW.getPitchPID()->setIntegralMinLimPct(static_cast<uint8_t>(val));
    ctx->stabilizeCLAW.getPitchPID()->setIntegralMaxLimPct(static_cast<uint8_t>(val));
    return true;
}
bool AMParamSetup::updateRollLimitAng(AttitudeManager* ctx, float val) {
    if (val < 0.0f || val > 45.0f) return false;
    ctx->stabilizeCLAW.setRollLimitAngle(val);
    return true;
}
bool AMParamSetup::updatePitchLimitAng(AttitudeManager* ctx, float val) {
    if (val < 0.0f || val > 45.0f) return false;
    ctx->stabilizeCLAW.setPitchLimitAngle(val);
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
