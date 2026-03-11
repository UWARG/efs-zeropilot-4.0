#include "fbwb_mapping.hpp"

FBWBMapping::FBWBMapping(float control_iter_period_s) noexcept :
    rollPID(0.0f, 0.0f, 0.0f,
        0.0f, OUTPUT_MIN, OUTPUT_MAX,
        INTEGRAL_MIN, INTEGRAL_MAX, control_iter_period_s),
    pitchPID(0.0f, 0.0f, 0.0f,
        0.0f, OUTPUT_MIN, OUTPUT_MAX,
        INTEGRAL_MIN, INTEGRAL_MAX, control_iter_period_s),
    totalEnergyPID(0.0f, 0.0f, 0.0f,
        0.0f, OUTPUT_MIN, OUTPUT_MAX,
        INTEGRAL_MIN, INTEGRAL_MAX, control_iter_period_s * OUTER_LOOP_DIVIDER),
    energyBalancePID(0.0f, 0.0f, 0.0f,
        0.0f, OUTPUT_MIN, OUTPUT_MAX,
        INTEGRAL_MIN, INTEGRAL_MAX, control_iter_period_s * OUTER_LOOP_DIVIDER),
    isInitialized(false),
    targetAltitude_m(0.0f),
    currentPitchSetpoint_rad(0.0f),
    currentThrottleOutput_pct(50.0f),
    outerLoopSchedulingCounter(0),
    dt_s(control_iter_period_s),
    yawRudderMixingConst(0.0f)
{
    rollPID.pidInitState();
    pitchPID.pidInitState();
    totalEnergyPID.pidInitState();
    energyBalancePID.pidInitState();
}

void FBWBMapping::setRollPIDConstants(float newKp, float newKi, float newKd, float newTau) noexcept {
    rollPID.setConstants(newKp, newKi, newKd, newTau);
}

void FBWBMapping::setPitchPIDConstants(float newKp, float newKi, float newKd, float newTau) noexcept {
    pitchPID.setConstants(newKp, newKi, newKd, newTau);
}

void FBWBMapping::setTotalEnergyPIDConstants(float newKp, float newKi, float newKd, float newTau) noexcept {
    totalEnergyPID.setConstants(newKp, newKi, newKd, newTau);
}

void FBWBMapping::setEnergyBalancePIDConstants(float newKp, float newKi, float newKd, float newTau) noexcept {
    energyBalancePID.setConstants(newKp, newKi, newKd, newTau);
}

void FBWBMapping::setYawRudderMixingConstant(float newMixingConst) noexcept {
    yawRudderMixingConst = newMixingConst;
}

void FBWBMapping::resetControlLoopState() noexcept {
    rollPID.pidInitState();
    pitchPID.pidInitState();
    totalEnergyPID.pidInitState();
    energyBalancePID.pidInitState();
}

void FBWBMapping::activateFlightMode() {
    resetControlLoopState();
    isInitialized = false;
    outerLoopSchedulingCounter = 0;
}

RCMotorControlMessage_t FBWBMapping::runControl(RCMotorControlMessage_t controlInputs, const DroneState_t &droneState) {
    // TODO: Add FBWB TECS logic, for now is a DirectMapping
    return controlInputs;
}
