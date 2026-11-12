#include <cmath>
#include "stabilize_mapping.hpp"
#include "unit_conversions.hpp"

StabilizeMapping::StabilizeMapping(float control_iter_period_s, AcroMapping &acro) noexcept : 
    rollPID(0.0f, 0.0f, 0.0f, 0.0f,
            OUTPUT_MIN, OUTPUT_MAX, 100,
            control_iter_period_s / ANGLE_LOOP_TO_INNER_LOOP_RATIO),
    pitchPID(0.0f, 0.0f, 0.0f, 0.0f,
            OUTPUT_MIN, OUTPUT_MAX, 100,
            control_iter_period_s / ANGLE_LOOP_TO_INNER_LOOP_RATIO),
    rollPitchLimitAngle(0.0f),
    acroCLAW(acro),
    decimationCounter(0),
    stabilizeRollCmd(STABILIZE_PID_OUTPUT_SHIFT),
    stabilizePitchCmd(STABILIZE_PID_OUTPUT_SHIFT) {
        (void)rollPID.pidInitState();
        (void)pitchPID.pidInitState();
}

// Setter for *roll* PID consts
void StabilizeMapping::setRollPIDConstants(float newKp, float newKi, float newKd, float newTau, uint8_t newIMaxPct) noexcept {
    (void)rollPID.setConstants(newKp, newKi, newKd, newTau, newIMaxPct);
}

// Setter for *pitch* PID consts
void StabilizeMapping::setPitchPIDConstants(float newKp, float newKi, float newKd, float newTau, uint8_t newIMaxPct) noexcept {
    (void)pitchPID.setConstants(newKp, newKi, newKd, newTau, newIMaxPct);
}

// Resetter for both roll and pitch PIDs (needed for unit testing)
void StabilizeMapping::resetControlLoopState() noexcept {
    (void)rollPID.pidInitState();
    (void)pitchPID.pidInitState();
    decimationCounter = 0;
    stabilizeRollCmd = STABILIZE_PID_OUTPUT_SHIFT;
    stabilizePitchCmd = STABILIZE_PID_OUTPUT_SHIFT;
}

// Setter for *rollLimitAngle* and *pitchLimitAngle* in rad
void StabilizeMapping::setRollPitchLimitAngle(float newRollPitchLimitAngle) noexcept {
    rollPitchLimitAngle = newRollPitchLimitAngle * ZP_UNITS::DEG_TO_RAD;
}

// Getter for PID objects
PID *StabilizeMapping::getRollPID() noexcept { return &rollPID; }
PID *StabilizeMapping::getPitchPID() noexcept { return &pitchPID; }

ZP_Error StabilizeMapping::activateFlightMode() {
    resetControlLoopState();
    acroCLAW.resetControlLoopState();
    return ZP_ERROR_OK;
}

// Main control mapping function for STABILIZE mode
ZP_Error StabilizeMapping::runControl(RCMotorControlMessage_t &controlOutput, RCMotorControlMessage_t controlInput, const DroneState_t &droneState) {
    ZP_Error result = ZP_ERROR_OK;

    // Outer angle loop runs once every ANGLE_LOOP_TO_INNER_LOOP_RATIO calls
    if (decimationCounter == 0) {
        // Setpoints: Maps [0, 100] to [-limit, +limit]
        float rollAngleSetpoint = ((controlInput.roll / MAX_RC_INPUT_VAL) * 2.0f - 1.0f) * rollPitchLimitAngle;
        float pitchAngleSetpoint = ((controlInput.pitch / MAX_RC_INPUT_VAL) * 2.0f - 1.0f) * rollPitchLimitAngle;

        float rollAngleMeasured = droneState.roll;
        float pitchAngleMeasured = droneState.pitch;

        // Run PID (output control efforts in [-1,1]), then scale back to RC controller range [0,100] for acro control loop
        float rollPidOut = 0.0f;
        float pitchPidOut = 0.0f;
        result |= rollPID.pidOutput(rollAngleSetpoint, rollAngleMeasured, rollPidOut);
        result |= pitchPID.pidOutput(pitchAngleSetpoint, pitchAngleMeasured, pitchPidOut);

        stabilizeRollCmd = (rollPidOut * STABILIZE_PID_OUTPUT_SCALE) + STABILIZE_PID_OUTPUT_SHIFT;
        stabilizePitchCmd = (pitchPidOut * STABILIZE_PID_OUTPUT_SCALE) + STABILIZE_PID_OUTPUT_SHIFT;
    }

    decimationCounter = (decimationCounter + 1) % ANGLE_LOOP_TO_INNER_LOOP_RATIO;

    RCMotorControlMessage_t acroInput = controlInput;
    acroInput.roll = stabilizeRollCmd;
    acroInput.pitch = stabilizePitchCmd;

    // Run acro control at the full AM loop rate
    result |= acroCLAW.runControl(controlOutput, acroInput, droneState);

    return result;
}
