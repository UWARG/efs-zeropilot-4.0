#include "fbwa_mapping.hpp"
#include "unit_conversions.hpp"
#include "zp_error.h"


FBWAMapping::FBWAMapping(float control_iter_period_s) noexcept :
    rollPID(0.0f, 0.0f, 0.0f, 0.0f,
        OUTPUT_MIN, OUTPUT_MAX, 100,
        control_iter_period_s),
    pitchPID(0.0f, 0.0f, 0.0f, 0.0f, 
        OUTPUT_MIN, OUTPUT_MAX, 100,
        control_iter_period_s),
    yawRudderMixingConst(0.0f),
    rollLimitRad(0.0f),
    pitchLimitMaxRad(0.0f),
    pitchLimitMinRad(0.0f)
{
    rollPID.pidInitState();
    pitchPID.pidInitState();
}

// Setter *roll* for PID consts
ZP_ERROR_e FBWAMapping::setRollPIDConstants(float newKp, float newKi, float newKd, float newTau, uint8_t newIMaxPct) noexcept {
    return rollPID.setConstants(newKp, newKi, newKd, newTau, newIMaxPct);
}

// Setter for *pitch* PID consts
ZP_ERROR_e FBWAMapping::setPitchPIDConstants(float newKp, float newKi, float newKd, float newTau, uint8_t newIMaxPct) noexcept {
    return pitchPID.setConstants(newKp, newKi, newKd, newTau, newIMaxPct);
}

// Resetter for both roll and pitch PIDs
ZP_ERROR_e FBWAMapping::resetControlLoopState() noexcept {
    ZP_ERROR_e result = ZP_ERROR_OK;
    result |= rollPID.pidInitState();
    result |= pitchPID.pidInitState();
    return result;
}

// Setter for *yaw* rudder mixing const
void FBWAMapping::setYawRudderMixingConstant(float newMixingConst) noexcept {
    yawRudderMixingConst = newMixingConst;
    return ZP_ERROR_OK;
}

// Setter for *rollLimitDeg*
ZP_ERROR_e FBWAMapping::setRollLimitDeg(float newRollLimitDeg) noexcept {
    rollLimitRad = ZP_UNITS::deg2rad(newRollLimitDeg);
    return ZP_ERROR_OK;
}

// Setter for *pitchLimitMaxDeg*
ZP_ERROR_e FBWAMapping::setPitchLimitMaxDeg(float newPitchLimitMaxDeg) noexcept {
    pitchLimitMaxRad = ZP_UNITS::deg2rad(newPitchLimitMaxDeg);
    return ZP_ERROR_OK;
}

// Setter for *pitchLimitMinDeg*
ZP_ERROR_e FBWAMapping::setPitchLimitMinDeg(float newPitchLimitMinDeg) noexcept {
    pitchLimitMinRad = ZP_UNITS::deg2rad(newPitchLimitMinDeg);
    return ZP_ERROR_OK;
}

// Getter for PID objects
ZP_ERROR_e FBWAMapping::getRollPID(PID*& out_rollPID) noexcept { 
    out_rollPID = &rollPID; 
    return ZP_ERROR_OK;
}

ZP_ERROR_e FBWAMapping::getPitchPID(PID*& out_pitchPID) noexcept { 
    out_pitchPID = &pitchPID; 
    return ZP_ERROR_OK;
}

ZP_ERROR_e FBWAMapping::activateFlightMode() {
    return resetControlLoopState();
}

// Main control mapping function for FBWA mode
ZP_ERROR_e FBWAMapping::runControl(RCMotorControlMessage_t &controlOutput, const RCMotorControlMessage_t controlInput, const DroneState_t &droneState) {
    ZP_ERROR_e result = ZP_ERROR_OK;

    // Roll SP: Maps [0, 100] to [-limit, +limit]
    float rollSetpoint = ((controlInput.roll / MAX_RC_INPUT_VAL) * 2.0f - 1.0f) * rollLimitRad;

    // Pitch SP: Linear interpolation that ensures 50% is always 0, even with asymmetric limits
    float pitchSetpoint = (controlInput.pitch > (MAX_RC_INPUT_VAL / 2.0f)) 
        ? (controlInput.pitch / (MAX_RC_INPUT_VAL / 2.0f) - 1.0f) * pitchLimitMaxRad 
        : (1.0f - controlInput.pitch / (MAX_RC_INPUT_VAL / 2.0f)) * pitchLimitMinRad;

    // Get measured values from drone state (populated by IMU)
    float rollMeasured = droneState.roll;
    float pitchMeasured = droneState.pitch;

    // Currently, roll & pitch outputs receive absolute roll & pitch angles, not relative to current position.
    float rollOutput = 0.0f;
    float pitchOutput = 0.0f;
    
    // Accumulate errors from PID calculations
    result |= rollPID.pidOutput(rollSetpoint, rollMeasured, rollOutput);
    result |= pitchPID.pidOutput(pitchSetpoint, pitchMeasured, pitchOutput);

    controlOutput = controlInput;
    controlOutput.roll = (rollOutput * FBWA_PID_OUTPUT_SCALE) + FBWA_PID_OUTPUT_SHIFT; // setting desired roll angle, adding 50 to shift to [0,100] range
    controlOutput.pitch = (pitchOutput * FBWA_PID_OUTPUT_SCALE) + FBWA_PID_OUTPUT_SHIFT; // setting desired pitch angle, adding 50 to shift to [0,100] range


    // Yaw control via rudder mixing
    float aileronSignalCentered = controlOutput.roll - (MAX_RC_INPUT_VAL / 2.0f); // Centering aileron signal around 0 for mixing calculation
    controlOutput.yaw += (yawRudderMixingConst * aileronSignalCentered); // Yaw adjustment based on roll PID output and mixing constant
    if (controlOutput.yaw < 0.0f) {
        controlOutput.yaw = 0.0f; // Ensuring yaw does not go below 0
    } else if (controlOutput.yaw > MAX_RC_INPUT_VAL) {
        controlOutput.yaw = MAX_RC_INPUT_VAL; // Ensuring yaw does not exceed max RC input value
    }

    return result;
}