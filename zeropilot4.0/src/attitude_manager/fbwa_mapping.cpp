#include "fbwa_mapping.hpp"
#include "unit_conversions.hpp"
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
    rollPID.pidInitState();
    pitchPID.pidInitState();
}

// Setter *roll* for PID consts
void FBWAMapping::setRollPIDConstants(float newKp, float newKi, float newKd, float newTau, uint8_t newIMaxPct) noexcept {
    rollPID.setConstants(newKp, newKi, newKd, newTau, newIMaxPct);
}

// Setter for *pitch* PID consts
void FBWAMapping::setPitchPIDConstants(float newKp, float newKi, float newKd, float newTau, uint8_t newIMaxPct) noexcept {
    pitchPID.setConstants(newKp, newKi, newKd, newTau, newIMaxPct);
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
void FBWAMapping::resetControlLoopState() noexcept {
    rollPID.pidInitState();
    pitchPID.pidInitState();
    prevRollSetpoint = 0.0f;
    prevPitchSetpoint = 0.0f;
    prevFilteredRollRate = 0.0f;
    prevFilteredPitchRate = 0.0f;
}

// Setter for *yaw* rudder mixing const
void FBWAMapping::setYawRudderMixingConstant(float newMixingConst) noexcept {
    yawRudderMixingConst = newMixingConst;
}

// Setter for *rollLimitDeg*
void FBWAMapping::setRollLimitDeg(float newRollLimitDeg) noexcept {
    rollLimitRad = ZP_UNITS::deg2rad(newRollLimitDeg);
}

// Setter for *pitchLimitMaxDeg*
void FBWAMapping::setPitchLimitMaxDeg(float newPitchLimitMaxDeg) noexcept {
    pitchLimitMaxRad = ZP_UNITS::deg2rad(newPitchLimitMaxDeg);
}

// Setter for *pitchLimitMinDeg*
void FBWAMapping::setPitchLimitMinDeg(float newPitchLimitMinDeg) noexcept {
    pitchLimitMinRad = ZP_UNITS::deg2rad(newPitchLimitMinDeg);
}

// Getter for PID objects
PID *FBWAMapping::getRollPID() noexcept { return &rollPID; }
PID *FBWAMapping::getPitchPID() noexcept { return &pitchPID; }

void FBWAMapping::activateFlightMode() {
    resetControlLoopState();
}

// Main control mapping function for FBWA mode
RCMotorControlMessage_t FBWAMapping::runControl(RCMotorControlMessage_t controlInputs, const DroneState_t &droneState){
    // Roll SP: Maps [0, 100] to [-limit, +limit]
    float rollSetpoint = ((controlInputs.roll / MAX_RC_INPUT_VAL) * 2.0f - 1.0f) * rollLimitRad;

    // Pitch SP: Linear interpolation that ensures 50% is always 0, even with asymmetric limits
    float pitchSetpoint = (controlInputs.pitch > (MAX_RC_INPUT_VAL / 2.0f)) 
        ? (controlInputs.pitch / (MAX_RC_INPUT_VAL / 2.0f) - 1.0f) * pitchLimitMaxRad 
        : (1.0f - controlInputs.pitch / (MAX_RC_INPUT_VAL / 2.0f)) * pitchLimitMinRad;

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
    float rollPIDOut = rollPID.pidOutput(rollSetpoint, rollMeasured);
    float pitchPIDOut = pitchPID.pidOutput(pitchSetpoint, pitchMeasured);

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
    controlInputs.roll = (rollTotalOut * FBWA_PID_OUTPUT_SCALE) + FBWA_PID_OUTPUT_SHIFT; // setting desired roll angle, adding 50 to shift to [0,100] range
    controlInputs.pitch = (pitchTotalOut * FBWA_PID_OUTPUT_SCALE) + FBWA_PID_OUTPUT_SHIFT; // setting desired pitch angle, adding 50 to shift to [0,100] range

    // Yaw control via rudder mixing
    float aileronSignalCentered = controlInputs.roll - (MAX_RC_INPUT_VAL / 2.0f); // Centering aileron signal around 0 for mixing calculation
    controlInputs.yaw += (yawRudderMixingConst * aileronSignalCentered); // Yaw adjustment based on roll PID output and mixing constant
    if (controlInputs.yaw < 0.0f) {
        controlInputs.yaw = 0.0f; // Ensuring yaw does not go below 0
    } else if (controlInputs.yaw > MAX_RC_INPUT_VAL) {
        controlInputs.yaw = MAX_RC_INPUT_VAL; // Ensuring yaw does not exceed max RC input value
    }

    return controlInputs;
}
