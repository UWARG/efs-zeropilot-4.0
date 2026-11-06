#include "fbwa_mapping.hpp"
#include "unit_conversions.hpp"
#include "zp_error.h"
#include <algorithm>

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

FBWAMapping::FBWAMapping(float control_iter_period_s) noexcept :
    controlIterPeriod(control_iter_period_s),
    rollPID(0.0f, 0.0f, 0.0f, 0.0f,
        OUTPUT_MIN, OUTPUT_MAX, 100,
        control_iter_period_s),
    pitchPID(0.0f, 0.0f, 0.0f, 0.0f, 
        OUTPUT_MIN, OUTPUT_MAX, 100,
        control_iter_period_s),
    rollFF(0.0f),
    pitchFF(0.0f),
    // alpha = (2pi * fc * dt) / (1 + 2pi * fc * dt)
    ffLpfAlpha((2 * M_PI * FF_LPF_CUTOFF_FREQ * control_iter_period_s) /
                (1 + (2 * M_PI * FF_LPF_CUTOFF_FREQ * control_iter_period_s))),
    yawRudderMixingConst(0.0f),
    rollLimitRad(0.0f),
    pitchLimitMaxRad(0.0f),
    pitchLimitMinRad(0.0f),
    prevRollSetpoint(0.0f),
    prevPitchSetpoint(0.0f),
    prevFilteredRollRate(0.0f),
    prevFilteredPitchRate(0.0f)
{
    (void)rollPID.pidInitState();
    (void)pitchPID.pidInitState();
}

// Setter *roll* for PID consts
ZP_Error FBWAMapping::setRollPIDConstants(float newKp, float newKi, float newKd, float newTau, uint8_t newIMaxPct) noexcept {
    ZP_Error result = rollPID.setConstants(newKp, newKi, newKd, newTau, newIMaxPct);
    return result;
}

// Setter for *pitch* PID consts
ZP_Error FBWAMapping::setPitchPIDConstants(float newKp, float newKi, float newKd, float newTau, uint8_t newIMaxPct) noexcept {
    ZP_Error result = pitchPID.setConstants(newKp, newKi, newKd, newTau, newIMaxPct);
    return result;
}

// Setter for *roll* FF const
void FBWAMapping::setRollFFConstant(float newRollFFConst) noexcept {
    rollFF = newRollFFConst;
}

// Setter for *pitch* FF const
void FBWAMapping::setPitchFFConstant(float newPitchFFConst) noexcept {
    pitchFF = newPitchFFConst;
}

// Resetter for both roll and pitch PIDs (needed for unit testing)
ZP_Error FBWAMapping::resetControlLoopState() noexcept {
    ZP_Error result = ZP_ERROR_OK;
    result |= rollPID.pidInitState();
    result |= pitchPID.pidInitState();
    prevRollSetpoint = 0.0f;
    prevPitchSetpoint = 0.0f;
    prevFilteredRollRate = 0.0f;
    prevFilteredPitchRate = 0.0f;
    return result;
}

// Setter for *yaw* rudder mixing const
ZP_Error FBWAMapping::setYawRudderMixingConstant(float newMixingConst) noexcept {
    yawRudderMixingConst = newMixingConst;
    return ZP_ERROR_OK;
}

// Setter for *rollLimitDeg*
ZP_Error FBWAMapping::setRollLimitDeg(float newRollLimitDeg) noexcept {
    rollLimitRad = ZP_UNITS::deg2rad(newRollLimitDeg);
    return ZP_ERROR_OK;
}

// Setter for *pitchLimitMaxDeg*
ZP_Error FBWAMapping::setPitchLimitMaxDeg(float newPitchLimitMaxDeg) noexcept {
    pitchLimitMaxRad = ZP_UNITS::deg2rad(newPitchLimitMaxDeg);
    return ZP_ERROR_OK;
}

// Setter for *pitchLimitMinDeg*
ZP_Error FBWAMapping::setPitchLimitMinDeg(float newPitchLimitMinDeg) noexcept {
    pitchLimitMinRad = ZP_UNITS::deg2rad(newPitchLimitMinDeg);
    return ZP_ERROR_OK;
}

// Getter for PID objects
ZP_Error FBWAMapping::getRollPID(PID*& out_rollPID) noexcept { 
    out_rollPID = &rollPID; 
    return ZP_ERROR_OK;
}

ZP_Error FBWAMapping::getPitchPID(PID*& out_pitchPID) noexcept { 
    out_pitchPID = &pitchPID; 
    return ZP_ERROR_OK;
}

ZP_Error FBWAMapping::activateFlightMode() {
    return resetControlLoopState();
}

// Main control mapping function for FBWA mode
ZP_Error FBWAMapping::runControl(RCMotorControlMessage_t &controlOutput, const RCMotorControlMessage_t controlInput, const DroneState_t &droneState) {
    ZP_Error result = ZP_ERROR_OK;

    // Roll SP: Maps [0, 100] to [-limit, +limit]
    float rollSetpoint = ((controlInput.roll / MAX_RC_INPUT_VAL) * 2.0f - 1.0f) * rollLimitRad;

    // Pitch SP: Linear interpolation that ensures 50% is always 0, even with asymmetric limits
    float pitchSetpoint = (controlInput.pitch > (MAX_RC_INPUT_VAL / 2.0f)) 
        ? (controlInput.pitch / (MAX_RC_INPUT_VAL / 2.0f) - 1.0f) * pitchLimitMaxRad 
        : (1.0f - controlInput.pitch / (MAX_RC_INPUT_VAL / 2.0f)) * pitchLimitMinRad;

    // Get measured values from drone state (populated by IMU)
    float rollMeasured = droneState.roll;
    float pitchMeasured = droneState.pitch;

    // Calculate raw roll/pitch SP rates
    float rawRollSetpointRate = (rollSetpoint - prevRollSetpoint) / controlIterPeriod;
    float rawPitchSetpointRate = (pitchSetpoint - prevPitchSetpoint) / controlIterPeriod;

    // Apply First-Order Low-Pass Filter
    float filteredRollRate = (ffLpfAlpha * rawRollSetpointRate) + ((1.0f - ffLpfAlpha) * prevFilteredRollRate);
    float filteredPitchRate = (ffLpfAlpha * rawPitchSetpointRate) + ((1.0f - ffLpfAlpha) * prevFilteredPitchRate);

    // Save current SP values and filtered rates into prev trackers
    prevRollSetpoint = rollSetpoint;
    prevPitchSetpoint = pitchSetpoint;
    prevFilteredRollRate = filteredRollRate;
    prevFilteredPitchRate = filteredPitchRate;

    // Calculate PID outputs for roll/pitch
    float rollPIDOut = 0.0f;
    float pitchPIDOut = 0.0f;
    result |= rollPID.pidOutput(rollSetpoint, rollMeasured, rollPIDOut);
    result |= pitchPID.pidOutput(pitchSetpoint, pitchMeasured, pitchPIDOut);

    // Add feedforward term for responsiveness using the filtered rates
    float rollTotalOut = rollPIDOut + (rollFF * filteredRollRate);
    float pitchTotalOut = pitchPIDOut + (pitchFF * filteredPitchRate);

    // Clamp total roll output to [-1.0, 1.0] before shifting/scaling
    if (rollTotalOut > OUTPUT_MAX) rollTotalOut = OUTPUT_MAX;
    else if (rollTotalOut < OUTPUT_MIN) rollTotalOut = OUTPUT_MIN;

    // Clamp total pitch output to [-1.0, 1.0] before shifting/scaling
    if (pitchTotalOut > OUTPUT_MAX) pitchTotalOut = OUTPUT_MAX;
    else if (pitchTotalOut < OUTPUT_MIN) pitchTotalOut = OUTPUT_MIN;

    // Set output signals
    controlOutput = controlInput;
    controlOutput.roll = (rollTotalOut * FBWA_PID_OUTPUT_SCALE) + FBWA_PID_OUTPUT_SHIFT; // setting desired roll angle, adding 50 to shift to [0,100] range
    controlOutput.pitch = (pitchTotalOut * FBWA_PID_OUTPUT_SCALE) + FBWA_PID_OUTPUT_SHIFT; // setting desired pitch angle, adding 50 to shift to [0,100] range


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