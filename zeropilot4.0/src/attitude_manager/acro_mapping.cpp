#include "acro_mapping.hpp"
#include "unit_conversions.hpp"

ACROMapping::ACROMapping(float control_iter_period_s) noexcept :
    rollPID(0.0f, 0.0f, 0.0f, 0.0f,
        OUTPUT_MIN, OUTPUT_MAX, 100,
        control_iter_period_s),
    pitchPID(0.0f, 0.0f, 0.0f, 0.0f, 
        OUTPUT_MIN, OUTPUT_MAX, 100,
        control_iter_period_s),
    yawPID(0.0f, 0.0f, 0.0f, 0.0f, 
    OUTPUT_MIN, OUTPUT_MAX, 100,
    control_iter_period_s), 
    rollLimitRad(0.0f),
    pitchLimitRad(0.0f),
    yawLimitRad(0.0f)
{
    rollPID.pidInitState();
    pitchPID.pidInitState();
    yawPID.pidInitState();
}

// Setter *roll* for PID consts
void ACROMapping::setRollPIDConstants(float newKp, float newKi, float newKd, float newTau, uint8_t newIMaxPct) noexcept {
    rollPID.setConstants(newKp, newKi, newKd, newTau, newIMaxPct);
}

// Setter for *pitch* PID consts
void ACROMapping::setPitchPIDConstants(float newKp, float newKi, float newKd, float newTau, uint8_t newIMaxPct) noexcept {
    pitchPID.setConstants(newKp, newKi, newKd, newTau, newIMaxPct);
}

// Setter for *yaw* PID consts
void ACROMapping::setYawPIDConstants(float newKp, float newKi, float newKd, float newTau, uint8_t newIMaxPct) noexcept {
    yawPID.setConstants(newKp, newKi, newKd, newTau, newIMaxPct);
}

// Resetter for both roll and pitch PIDs (needed for unit testing)
void ACROMapping::resetControlLoopState() noexcept {
    rollPID.pidInitState();
    pitchPID.pidInitState();
    yawPID.pidInitState();
}

// Setter for *rollLimitDeg*
void ACROMapping::setRollLimitDeg(float newRollLimitDeg) noexcept {
    rollLimitRad = ZP_UNITS::deg2rad(newRollLimitDeg);
}

// Setter for *pitchLimitRad*
void ACROMapping::setPitchLimitDeg(float newPitchLimitDeg) noexcept {
    pitchLimitRad = ZP_UNITS::deg2rad(newPitchLimitDeg);
}

// Setter for *rollLimitDeg*
void ACROMapping::setYawLimitDeg(float newYawLimitDeg) noexcept {
    yawLimitRad = ZP_UNITS::deg2rad(newYawLimitDeg);
}

// Getter for PID objects
PID *ACROMapping::getRollPID() noexcept { return &rollPID; }
PID *ACROMapping::getPitchPID() noexcept { return &pitchPID; }
PID *ACROMapping::getYawPID() noexcept { return &yawPID; }

void ACROMapping::activateFlightMode() {
    resetControlLoopState();
}

// Main control mapping function for FBWA mode
RCMotorControlMessage_t ACROMapping::runControl(RCMotorControlMessage_t controlInputs, const DroneState_t &droneState){
    // Setpoints: Maps [0, 100] to [-limit, +limit]
    float rollSetpoint = ((controlInputs.roll / MAX_RC_INPUT_VAL) * 2.0f - 1.0f) * rollLimitRad;
    float pitchSetpoint = ((controlInputs.pitch / MAX_RC_INPUT_VAL) * 2.0f - 1.0f) * pitchLimitRad;
    float yawSetpoint = ((controlInputs.yaw / MAX_RC_INPUT_VAL) * 2.0f - 1.0f) * yawLimitRad;

    // Get measured values from drone state (populated by IMU)
    float rollMeasured = droneState.roll;
    float pitchMeasured = droneState.pitch;
    float yawMeasured = droneState.yaw;

    // Run PID, outputs are absolute angles
    float rollOutput = rollPID.pidOutput(rollSetpoint, rollMeasured);
    float pitchOutput = pitchPID.pidOutput(pitchSetpoint, pitchMeasured);
    float yawOutput = pitchPID.pidOutput(yawSetpoint, pitchMeasured);

    // Convert absolute angles to delta values from current position in [0,100] range 
    controlInputs.roll = (rollOutput * FBWA_PID_OUTPUT_SCALE) + FBWA_PID_OUTPUT_SHIFT; 
    controlInputs.pitch = (pitchOutput * FBWA_PID_OUTPUT_SCALE) + FBWA_PID_OUTPUT_SHIFT; 
    controlInputs.yaw = (yawOutput * FBWA_PID_OUTPUT_SCALE) + FBWA_PID_OUTPUT_SHIFT; 

    return controlInputs;
}
