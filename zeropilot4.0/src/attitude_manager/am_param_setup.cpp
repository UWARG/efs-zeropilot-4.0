#include "am_param_setup.hpp"
#include "attitude_manager.hpp"
#include "zp_params.hpp"
#include "motor_functions.hpp"

static inline int usToPercent(float us) {
    return static_cast<int>((us - 1000.0f) / 10.0f);
}

AMParamSetup::AMParamSetup(AttitudeManager* am) : am(am) {}

ZP_ERROR_e AMParamSetup::loadAllParams() {
    ZP_ERROR_e result = ZP_ERROR_OK;

    // FBWA params
    float p, i, d, tau, imax, rddr, rllLim, ptchMax, ptchMin;
    
    result |= ZP_PARAM::get(ZP_PARAM_ID::RLL2SRV_P, p);
    result |= ZP_PARAM::get(ZP_PARAM_ID::RLL2SRV_I, i);
    result |= ZP_PARAM::get(ZP_PARAM_ID::RLL2SRV_D, d);
    result |= ZP_PARAM::get(ZP_PARAM_ID::RLL2SRV_TAU, tau);
    result |= ZP_PARAM::get(ZP_PARAM_ID::RLL2SRV_IMAX, imax);
    result |= am->fbwaCLAW.setRollPIDConstants(p, i, d, tau, static_cast<uint8_t>(imax));

    result |= ZP_PARAM::get(ZP_PARAM_ID::PTCH2SRV_P, p);
    result |= ZP_PARAM::get(ZP_PARAM_ID::PTCH2SRV_I, i);
    result |= ZP_PARAM::get(ZP_PARAM_ID::PTCH2SRV_D, d);
    result |= ZP_PARAM::get(ZP_PARAM_ID::PTCH2SRV_TAU, tau);
    result |= ZP_PARAM::get(ZP_PARAM_ID::PTCH2SRV_IMAX, imax);
    result |= am->fbwaCLAW.setPitchPIDConstants(p, i, d, tau, static_cast<uint8_t>(imax));

    result |= ZP_PARAM::get(ZP_PARAM_ID::KFF_RDDRMIX, rddr);
    result |= am->fbwaCLAW.setYawRudderMixingConstant(rddr);
    
    result |= ZP_PARAM::get(ZP_PARAM_ID::ROLL_LIMIT_DEG, rllLim);
    result |= am->fbwaCLAW.setRollLimitDeg(rllLim);
    
    result |= ZP_PARAM::get(ZP_PARAM_ID::PTCH_LIM_MAX_DEG, ptchMax);
    result |= am->fbwaCLAW.setPitchLimitMaxDeg(ptchMax);
    
    result |= ZP_PARAM::get(ZP_PARAM_ID::PTCH_LIM_MIN_DEG, ptchMin);
    result |= am->fbwaCLAW.setPitchLimitMinDeg(ptchMin);

    // Servo params
    auto loadMotor = [&](uint8_t ch, ZP_PARAM_ID trim, ZP_PARAM_ID min, ZP_PARAM_ID max, ZP_PARAM_ID rev, ZP_PARAM_ID func) {
        if (ch >= am->mainMotorGroup->motorCount) return ZP_ERROR_INVALID_PARAM;
        float t, mn, mx, rv, fn;
        ZP_ERROR_e inner = ZP_PARAM::get(trim, t);
        inner |= ZP_PARAM::get(min, mn);
        inner |= ZP_PARAM::get(max, mx);
        inner |= ZP_PARAM::get(rev, rv);
        inner |= ZP_PARAM::get(func, fn);
        
        if (inner == ZP_ERROR_OK) {
            MotorInstance_t* m = &am->mainMotorGroup->motors[ch];
            m->trim = usToPercent(t);
            m->min = usToPercent(mn);
            m->max = usToPercent(mx);
            m->isInverted = static_cast<int>(rv) != 0;
            m->function = static_cast<MotorFunction_e>(static_cast<int16_t>(fn));
        }
        return inner;
    };

    result |= loadMotor(0,  ZP_PARAM_ID::SERVO1_TRIM,  ZP_PARAM_ID::SERVO1_MIN,  ZP_PARAM_ID::SERVO1_MAX,  ZP_PARAM_ID::SERVO1_REVERSED,  ZP_PARAM_ID::SERVO1_FUNCTION);
    result |= loadMotor(1,  ZP_PARAM_ID::SERVO2_TRIM,  ZP_PARAM_ID::SERVO2_MIN,  ZP_PARAM_ID::SERVO2_MAX,  ZP_PARAM_ID::SERVO2_REVERSED,  ZP_PARAM_ID::SERVO2_FUNCTION);
    result |= loadMotor(2,  ZP_PARAM_ID::SERVO3_TRIM,  ZP_PARAM_ID::SERVO3_MIN,  ZP_PARAM_ID::SERVO3_MAX,  ZP_PARAM_ID::SERVO3_REVERSED,  ZP_PARAM_ID::SERVO3_FUNCTION);
    result |= loadMotor(3,  ZP_PARAM_ID::SERVO4_TRIM,  ZP_PARAM_ID::SERVO4_MIN,  ZP_PARAM_ID::SERVO4_MAX,  ZP_PARAM_ID::SERVO4_REVERSED,  ZP_PARAM_ID::SERVO4_FUNCTION);
    result |= loadMotor(4,  ZP_PARAM_ID::SERVO5_TRIM,  ZP_PARAM_ID::SERVO5_MIN,  ZP_PARAM_ID::SERVO5_MAX,  ZP_PARAM_ID::SERVO5_REVERSED,  ZP_PARAM_ID::SERVO5_FUNCTION);
    result |= loadMotor(5,  ZP_PARAM_ID::SERVO6_TRIM,  ZP_PARAM_ID::SERVO6_MIN,  ZP_PARAM_ID::SERVO6_MAX,  ZP_PARAM_ID::SERVO6_REVERSED,  ZP_PARAM_ID::SERVO6_FUNCTION);
    result |= loadMotor(6,  ZP_PARAM_ID::SERVO7_TRIM,  ZP_PARAM_ID::SERVO7_MIN,  ZP_PARAM_ID::SERVO7_MAX,  ZP_PARAM_ID::SERVO7_REVERSED,  ZP_PARAM_ID::SERVO7_FUNCTION);
    result |= loadMotor(7,  ZP_PARAM_ID::SERVO8_TRIM,  ZP_PARAM_ID::SERVO8_MIN,  ZP_PARAM_ID::SERVO8_MAX,  ZP_PARAM_ID::SERVO8_REVERSED,  ZP_PARAM_ID::SERVO8_FUNCTION);
    result |= loadMotor(8,  ZP_PARAM_ID::SERVO9_TRIM,  ZP_PARAM_ID::SERVO9_MIN,  ZP_PARAM_ID::SERVO9_MAX,  ZP_PARAM_ID::SERVO9_REVERSED,  ZP_PARAM_ID::SERVO9_FUNCTION);
    result |= loadMotor(9,  ZP_PARAM_ID::SERVO10_TRIM, ZP_PARAM_ID::SERVO10_MIN, ZP_PARAM_ID::SERVO10_MAX, ZP_PARAM_ID::SERVO10_REVERSED, ZP_PARAM_ID::SERVO10_FUNCTION);
    result |= loadMotor(10, ZP_PARAM_ID::SERVO11_TRIM, ZP_PARAM_ID::SERVO11_MIN, ZP_PARAM_ID::SERVO11_MAX, ZP_PARAM_ID::SERVO11_REVERSED, ZP_PARAM_ID::SERVO11_FUNCTION);
    result |= loadMotor(11, ZP_PARAM_ID::SERVO12_TRIM, ZP_PARAM_ID::SERVO12_MIN, ZP_PARAM_ID::SERVO12_MAX, ZP_PARAM_ID::SERVO12_REVERSED, ZP_PARAM_ID::SERVO12_FUNCTION);

    return result;
}

// Macro to bind all 5 fields for a single servo channel
#define AM_PARAM_SETUP_BIND_SERVO_CB(N) \
    result |= ZP_PARAM::bindCallback(ZP_PARAM_ID::SERVO##N##_TRIM,     am, cbServoTrim<N-1>);     \
    result |= ZP_PARAM::bindCallback(ZP_PARAM_ID::SERVO##N##_MIN,      am, cbServoMin<N-1>);      \
    result |= ZP_PARAM::bindCallback(ZP_PARAM_ID::SERVO##N##_MAX,      am, cbServoMax<N-1>);      \
    result |= ZP_PARAM::bindCallback(ZP_PARAM_ID::SERVO##N##_REVERSED, am, cbServoReversed<N-1>); \
    result |= ZP_PARAM::bindCallback(ZP_PARAM_ID::SERVO##N##_FUNCTION, am, cbServoFunction<N-1>);

ZP_ERROR_e AMParamSetup::bindAllParamCallbacks() {
    ZP_ERROR_e result = ZP_ERROR_OK;

    // FBWA Binds
    result |= ZP_PARAM::bindCallback(ZP_PARAM_ID::RLL2SRV_P,        am, updatePIDRollKp);
    result |= ZP_PARAM::bindCallback(ZP_PARAM_ID::RLL2SRV_I,        am, updatePIDRollKi);
    result |= ZP_PARAM::bindCallback(ZP_PARAM_ID::RLL2SRV_D,        am, updatePIDRollKd);
    result |= ZP_PARAM::bindCallback(ZP_PARAM_ID::RLL2SRV_TAU,      am, updatePIDRollTau);
    result |= ZP_PARAM::bindCallback(ZP_PARAM_ID::RLL2SRV_IMAX,     am, updatePIDRollIMax);
    result |= ZP_PARAM::bindCallback(ZP_PARAM_ID::PTCH2SRV_P,       am, updatePIDPitchKp);
    result |= ZP_PARAM::bindCallback(ZP_PARAM_ID::PTCH2SRV_I,       am, updatePIDPitchKi);
    result |= ZP_PARAM::bindCallback(ZP_PARAM_ID::PTCH2SRV_D,       am, updatePIDPitchKd);
    result |= ZP_PARAM::bindCallback(ZP_PARAM_ID::PTCH2SRV_TAU,     am, updatePIDPitchTau);
    result |= ZP_PARAM::bindCallback(ZP_PARAM_ID::PTCH2SRV_IMAX,    am, updatePIDPitchIMax);
    result |= ZP_PARAM::bindCallback(ZP_PARAM_ID::KFF_RDDRMIX,       am, updateKffRddrmix);
    result |= ZP_PARAM::bindCallback(ZP_PARAM_ID::ROLL_LIMIT_DEG,    am, updateRollLimitDeg);
    result |= ZP_PARAM::bindCallback(ZP_PARAM_ID::PTCH_LIM_MAX_DEG, am, updatePitchLimMaxDeg);
    result |= ZP_PARAM::bindCallback(ZP_PARAM_ID::PTCH_LIM_MIN_DEG, am, updatePitchLimMinDeg);

    // Servo params
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

// FBWA callbacks

ZP_ERROR_e AMParamSetup::updatePIDRollKp(AttitudeManager* ctx, float val) {
    if (val < 0.0f) return ZP_ERROR_INVALID_PARAM;
    PID* pid = nullptr;
    ZP_ERROR_e res = ctx->fbwaCLAW.getRollPID(pid);
    if (res == ZP_ERROR_OK) pid->setKp(val);
    return res;
}

ZP_ERROR_e AMParamSetup::updatePIDRollKi(AttitudeManager* ctx, float val) {
    if (val < 0.0f) return ZP_ERROR_INVALID_PARAM;
    PID* pid = nullptr;
    ZP_ERROR_e res = ctx->fbwaCLAW.getRollPID(pid);
    if (res == ZP_ERROR_OK) pid->setKi(val);
    return res;
}

ZP_ERROR_e AMParamSetup::updatePIDRollKd(AttitudeManager* ctx, float val) {
    if (val < 0.0f) return ZP_ERROR_INVALID_PARAM;
    PID* pid = nullptr;
    ZP_ERROR_e res = ctx->fbwaCLAW.getRollPID(pid);
    if (res == ZP_ERROR_OK) pid->setKd(val);
    return res;
}

ZP_ERROR_e AMParamSetup::updatePIDRollTau(AttitudeManager* ctx, float val) {
    if (val < 0.0f) return ZP_ERROR_INVALID_PARAM;
    PID* pid = nullptr;
    ZP_ERROR_e res = ctx->fbwaCLAW.getRollPID(pid);
    if (res == ZP_ERROR_OK) pid->setTau(val);
    return res;
}

ZP_ERROR_e AMParamSetup::updatePIDRollIMax(AttitudeManager* ctx, float val) {
    if (val < 0.0f || val > 100.0f) return ZP_ERROR_INVALID_PARAM;
    PID* pid = nullptr;
    ZP_ERROR_e res = ctx->fbwaCLAW.getRollPID(pid);
    if (res == ZP_ERROR_OK) {
        pid->setIntegralMinLimPct(static_cast<uint8_t>(val));
        pid->setIntegralMaxLimPct(static_cast<uint8_t>(val));
    }
    return res;
}

ZP_ERROR_e AMParamSetup::updatePIDPitchKp(AttitudeManager* ctx, float val) {
    if (val < 0.0f) return ZP_ERROR_INVALID_PARAM;
    PID* pid = nullptr;
    ZP_ERROR_e res = ctx->fbwaCLAW.getPitchPID(pid);
    if (res == ZP_ERROR_OK) pid->setKp(val);
    return res;
}

ZP_ERROR_e AMParamSetup::updatePIDPitchKi(AttitudeManager* ctx, float val) {
    if (val < 0.0f) return ZP_ERROR_INVALID_PARAM;
    PID* pid = nullptr;
    ZP_ERROR_e res = ctx->fbwaCLAW.getPitchPID(pid);
    if (res == ZP_ERROR_OK) pid->setKi(val);
    return res;
}

ZP_ERROR_e AMParamSetup::updatePIDPitchKd(AttitudeManager* ctx, float val) {
    if (val < 0.0f) return ZP_ERROR_INVALID_PARAM;
    PID* pid = nullptr;
    ZP_ERROR_e res = ctx->fbwaCLAW.getPitchPID(pid);
    if (res == ZP_ERROR_OK) pid->setKd(val);
    return res;
}

ZP_ERROR_e AMParamSetup::updatePIDPitchTau(AttitudeManager* ctx, float val) {
    if (val < 0.0f) return ZP_ERROR_INVALID_PARAM;
    PID* pid = nullptr;
    ZP_ERROR_e res = ctx->fbwaCLAW.getPitchPID(pid);
    if (res == ZP_ERROR_OK) pid->setTau(val);
    return res;
}

ZP_ERROR_e AMParamSetup::updatePIDPitchIMax(AttitudeManager* ctx, float val) {
    if (val < 0.0f || val > 100.0f) return ZP_ERROR_INVALID_PARAM;
    PID* pid = nullptr;
    ZP_ERROR_e res = ctx->fbwaCLAW.getPitchPID(pid);
    if (res == ZP_ERROR_OK) {
        pid->setIntegralMinLimPct(static_cast<uint8_t>(val));
        pid->setIntegralMaxLimPct(static_cast<uint8_t>(val));
    }
    return res;
}

ZP_ERROR_e AMParamSetup::updateKffRddrmix(AttitudeManager* ctx, float val) {
    if (val < 0.0f || val > 1.0f) return ZP_ERROR_INVALID_PARAM;
    return ctx->fbwaCLAW.setYawRudderMixingConstant(val);
}

ZP_ERROR_e AMParamSetup::updateRollLimitDeg(AttitudeManager* ctx, float val) {
    if (val < 0.0f || val > 90.0f) return ZP_ERROR_INVALID_PARAM;
    return ctx->fbwaCLAW.setRollLimitDeg(val);
}

ZP_ERROR_e AMParamSetup::updatePitchLimMaxDeg(AttitudeManager* ctx, float val) {
    if (val < 0.0f || val > 90.0f) return ZP_ERROR_INVALID_PARAM;
    return ctx->fbwaCLAW.setPitchLimitMaxDeg(val);
}

ZP_ERROR_e AMParamSetup::updatePitchLimMinDeg(AttitudeManager* ctx, float val) {
    if (val < -90.0f || val > 0.0f) return ZP_ERROR_INVALID_PARAM;
    return ctx->fbwaCLAW.setPitchLimitMinDeg(val);
}

// Servo field helpers

ZP_ERROR_e AMParamSetup::setServoTrim(AttitudeManager* ctx, uint8_t ch, float val) {
    if (ch >= ctx->mainMotorGroup->motorCount || val < 0.0f || val > 2000.0f) return ZP_ERROR_INVALID_PARAM;
    ctx->mainMotorGroup->motors[ch].trim = usToPercent(val);
    return ZP_ERROR_OK;
}

ZP_ERROR_e AMParamSetup::setServoMin(AttitudeManager* ctx, uint8_t ch, float val) {
    if (ch >= ctx->mainMotorGroup->motorCount || val < 0.0f || val > 2000.0f) return ZP_ERROR_INVALID_PARAM;
    ctx->mainMotorGroup->motors[ch].min = usToPercent(val);
    return ZP_ERROR_OK;
}

ZP_ERROR_e AMParamSetup::setServoMax(AttitudeManager* ctx, uint8_t ch, float val) {
    if (ch >= ctx->mainMotorGroup->motorCount || val < 0.0f || val > 2000.0f) return ZP_ERROR_INVALID_PARAM;
    ctx->mainMotorGroup->motors[ch].max = usToPercent(val);
    return ZP_ERROR_OK;
}

ZP_ERROR_e AMParamSetup::setServoReversed(AttitudeManager* ctx, uint8_t ch, float val) {
    int v = static_cast<int>(val);
    if (ch >= ctx->mainMotorGroup->motorCount || (v != 0 && v != 1)) return ZP_ERROR_INVALID_PARAM;
    ctx->mainMotorGroup->motors[ch].isInverted = (v != 0);
    return ZP_ERROR_OK;
}

ZP_ERROR_e AMParamSetup::setServoFunction(AttitudeManager* ctx, uint8_t ch, float val) {
    if (ch >= ctx->mainMotorGroup->motorCount) return ZP_ERROR_INVALID_PARAM;
    ctx->mainMotorGroup->motors[ch].function = static_cast<MotorFunction_e>(static_cast<int16_t>(val));
    return ZP_ERROR_OK;
}