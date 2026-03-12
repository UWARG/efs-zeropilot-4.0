#include "fbwb_mapping.hpp"

FBWBMapping::FBWBMapping(float control_iter_period_s) noexcept :
    FBWAMapping(control_iter_period_s),
    totalEnergyPID(0.0f, 0.0f, 0.0f,
        0.0f, PID_OUTPUT_MIN, PID_OUTPUT_MAX,
        PID_INTEGRAL_MIN, PID_INTEGRAL_MAX,
        control_iter_period_s * OUTER_LOOP_DIVIDER),
    energyBalancePID(0.0f, 0.0f, 0.0f,
        0.0f, PID_OUTPUT_MIN, PID_OUTPUT_MAX,
        PID_INTEGRAL_MIN, PID_INTEGRAL_MAX,
        control_iter_period_s * OUTER_LOOP_DIVIDER),
    isInitialized(false),
    targetAltitude_m(0.0f),
    currentPitchSetpoint(0.0f),
    currentThrottleOutput_pct(50.0f),
    outerLoopSchedulingCounter(0),
    dt_s(control_iter_period_s)
{
    totalEnergyPID.pidInitState();
    energyBalancePID.pidInitState();
}

void FBWBMapping::setTotalEnergyPIDConstants(float newKp, float newKi, float newKd, float newTau) noexcept {
    totalEnergyPID.setConstants(newKp, newKi, newKd, newTau);
}

void FBWBMapping::setEnergyBalancePIDConstants(float newKp, float newKi, float newKd, float newTau) noexcept {
    energyBalancePID.setConstants(newKp, newKi, newKd, newTau);
}

void FBWBMapping::resetControlLoopState() noexcept {
    FBWAMapping::resetControlLoopState();
    totalEnergyPID.pidInitState();
    energyBalancePID.pidInitState();
}

void FBWBMapping::activateFlightMode() {
    FBWAMapping::activateFlightMode();
    resetControlLoopState();
    isInitialized = false;
    outerLoopSchedulingCounter = 0;
}

RCMotorControlMessage_t FBWBMapping::runControl(RCMotorControlMessage_t controlInputs, const DroneState_t &droneState) {
    // TODO: Add FBWB TECS logic, for now is a DirectMapping
    return controlInputs;
}

float FBWBMapping::calculateSpecificKE(float airspeed_mps) {
    return (airspeed_mps * airspeed_mps) / (2.0f * GRAVITY_MSS);
}
