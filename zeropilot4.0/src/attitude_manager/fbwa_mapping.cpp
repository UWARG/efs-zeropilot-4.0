#include "fbwa_mapping.hpp"
#include "unit_conversions.hpp"

FBWAMapping::FBWAMapping(float control_iter_period_s) noexcept :
    rollPID(0.0f, 0.0f, 0.0f,
        0.0f, OUTPUT_MIN, OUTPUT_MAX,
        INTEGRAL_MIN, INTEGRAL_MAX, control_iter_period_s),
    pitchPID(0.0f, 0.0f, 0.0f,
        0.0f, OUTPUT_MIN, OUTPUT_MAX,
        INTEGRAL_MIN, INTEGRAL_MAX, control_iter_period_s),
    yawRudderMixingConst(0.0f),
    rollLimitRad(0.0f),
    pitchLimitMaxRad(0.0f),
    pitchLimitMinRad(0.0f)
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

// Resetter for both roll and pitch PIDs
void FBWAMapping::resetControlLoopState() noexcept {
    rollPID.pidInitState();
    pitchPID.pidInitState();
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

    // Currently, roll & pitch outputs receive absolute roll & pitch angles, not relative to current position.
    float rollOutput = rollPID.pidOutput(rollSetpoint, rollMeasured);
    float pitchOutput = pitchPID.pidOutput(pitchSetpoint, pitchMeasured);

    controlInputs.roll = (rollOutput * FBWA_PID_OUTPUT_SCALE) + FBWA_PID_OUTPUT_SHIFT; // setting desired roll angle, adding 50 to shift to [0,100] range
    controlInputs.pitch = (pitchOutput * FBWA_PID_OUTPUT_SCALE) + FBWA_PID_OUTPUT_SHIFT; // setting desired pitch angle, adding 50 to shift to [0,100] range

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
