#include <cmath>
#include "stabilize_mapping.hpp"
#include "unit_conversions.hpp"

StabilizeMapping::StabilizeMapping(float stabilize_control_iter_period_s, float acro_control_iter_period_s, AcroMapping &acro) noexcept : 
    rollPID(0.0f, 0.0f, 0.0f, 0.0f,
            OUTPUT_MIN, OUTPUT_MAX, 100,
            stabilize_control_iter_period_s),
    pitchPID(0.0f, 0.0f, 0.0f, 0.0f,
            OUTPUT_MIN, OUTPUT_MAX, 100,
            stabilize_control_iter_period_s),
    rollPitchLimitAngle(0.0f),
    acroCLAW(acro),
    decimationFactor(computeDecimation(stabilize_control_iter_period_s, acro_control_iter_period_s)),
    decimationCounter(0),
    stabilizeRollCmd(ACRO_PID_OUTPUT_SHIFT),
    stabilizePitchCmd(ACRO_PID_OUTPUT_SHIFT) {
        rollPID.pidInitState();
        pitchPID.pidInitState();
}

// Setter *roll* for PID consts
void StabilizeMapping::setRollPIDConstants(float newKp, float newKi, float newKd, float newTau, uint8_t newIMaxPct) noexcept {
    rollPID.setConstants(newKp, newKi, newKd, newTau, newIMaxPct);
}

// Setter for *pitch* PID consts
void StabilizeMapping::setPitchPIDConstants(float newKp, float newKi, float newKd, float newTau, uint8_t newIMaxPct) noexcept {
    pitchPID.setConstants(newKp, newKi, newKd, newTau, newIMaxPct);
}

// Resetter for both roll and pitch PIDs (needed for unit testing)
void StabilizeMapping::resetControlLoopState() noexcept {
    rollPID.pidInitState();
    pitchPID.pidInitState();
    decimationCounter = 0;
    stabilizeRollCmd = ACRO_PID_OUTPUT_SHIFT;
    stabilizePitchCmd = ACRO_PID_OUTPUT_SHIFT;
}

// Setter for *rollLimitAngle* and *pitchLimitAngle* in rad
void StabilizeMapping::setRollPitchLimitAngle(float newRollPitchLimitAngle) noexcept {
    rollPitchLimitAngle = newRollPitchLimitAngle * ZP_UNITS::DEG_TO_RAD;
}

// Getter for PID objects
PID *StabilizeMapping::getRollPID() noexcept { return &rollPID; }
PID *StabilizeMapping::getPitchPID() noexcept { return &pitchPID; }

void StabilizeMapping::activateFlightMode() {
    resetControlLoopState();
    acroCLAW.resetControlLoopState();
}

// Main control mapping function for STABILIZE mode
RCMotorControlMessage_t StabilizeMapping::runControl(RCMotorControlMessage_t controlInputs, const DroneState_t &droneState) {
    // Outer angle loop runs once every decimationFactor calls
    if (decimationCounter == 0) {
        // Setpoints: Maps [0, 100] to [-limit, +limit]
        float rollAngleSetpoint = ((controlInputs.roll / MAX_RC_INPUT_VAL) * 2.0f - 1.0f) * rollPitchLimitAngle;
        float pitchAngleSetpoint = ((controlInputs.pitch / MAX_RC_INPUT_VAL) * 2.0f - 1.0f) * rollPitchLimitAngle;

        float rollAngleMeasured = droneState.roll;
        float pitchAngleMeasured = droneState.pitch;

        // Run PID (output control efforts in [-1,1]), then scale back to RC controller range [0,100] for acro control loop
        stabilizeRollCmd = (rollPID.pidOutput(rollAngleSetpoint, rollAngleMeasured) * ACRO_PID_OUTPUT_SCALE) + ACRO_PID_OUTPUT_SHIFT;
        stabilizePitchCmd = (pitchPID.pidOutput(pitchAngleSetpoint, pitchAngleMeasured) * ACRO_PID_OUTPUT_SCALE) + ACRO_PID_OUTPUT_SHIFT;
    }
    decimationCounter = (decimationCounter + 1) % decimationFactor;

    controlInputs.roll = stabilizeRollCmd;
    controlInputs.pitch = stabilizePitchCmd;

    // Run acro control at the full loop rate
    controlInputs = acroCLAW.runControl(controlInputs, droneState);

    return controlInputs;
}
