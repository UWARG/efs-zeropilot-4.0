#include <cmath>
#include "acro_mapping.hpp"
#include "unit_conversions.hpp"

AcroMapping::AcroMapping(float control_iter_period_s) noexcept : 
    rollPID(0.0f, 0.0f, 0.0f, 0.0f,
        OUTPUT_MIN, OUTPUT_MAX, 100,
        control_iter_period_s),
    pitchPID(0.0f, 0.0f, 0.0f, 0.0f,
        OUTPUT_MIN, OUTPUT_MAX, 100,
        control_iter_period_s),
    yawPID(0.0f, 0.0f, 0.0f, 0.0f,
        OUTPUT_MIN, OUTPUT_MAX, 100,
        control_iter_period_s),
    rollLimitRate(0.0f),
    pitchLimitRate(0.0f),
    yawLimitRate(0.0f) {
        rollPID.pidInitState();
        pitchPID.pidInitState();
        yawPID.pidInitState();
}

// Setter *roll* for PID consts
void AcroMapping::setRollPIDConstants(float newKp, float newKi, float newKd, float newTau, uint8_t newIMaxPct) noexcept {
    rollPID.setConstants(newKp, newKi, newKd, newTau, newIMaxPct);
}

// Setter for *pitch* PID consts
void AcroMapping::setPitchPIDConstants(float newKp, float newKi, float newKd, float newTau, uint8_t newIMaxPct) noexcept {
    pitchPID.setConstants(newKp, newKi, newKd, newTau, newIMaxPct);
}

// Setter for *yaw* PID consts
void AcroMapping::setYawPIDConstants(float newKp, float newKi, float newKd, float newTau, uint8_t newIMaxPct) noexcept {
    yawPID.setConstants(newKp, newKi, newKd, newTau, newIMaxPct);
}

// Resetter for both roll and pitch PIDs (needed for unit testing)
void AcroMapping::resetControlLoopState() noexcept {
    rollPID.pidInitState();
    pitchPID.pidInitState();
    yawPID.pidInitState();
}

// Setter for *rollLimitRate* in rad / s
void AcroMapping::setRollLimitRate(float newRollLimitRate) noexcept {
    rollLimitRate = newRollLimitRate;
}

// Setter for *pitchLimitRate* in rad / s
void AcroMapping::setPitchLimitRate(float newPitchLimitRate) noexcept {
    pitchLimitRate = newPitchLimitRate;
}

// Setter for *yawLimitRate* in rad / s
void AcroMapping::setYawLimitRate(float newYawLimitRate) noexcept {
    yawLimitRate = newYawLimitRate;
}

// Getter for PID objects
PID *AcroMapping::getRollPID() noexcept { return &rollPID; }
PID *AcroMapping::getPitchPID() noexcept { return &pitchPID; }
PID *AcroMapping::getYawPID() noexcept { return &yawPID; }

void AcroMapping::activateFlightMode() {
    resetControlLoopState();
}

// Main control mapping function for ACRO mode
RCMotorControlMessage_t AcroMapping::runControl(RCMotorControlMessage_t controlInputs, const DroneState_t &droneState) {
    // Setpoints: Maps [0, 100] to [-limit, +limit]
    float rollRateSetpoint = ((controlInputs.roll / MAX_RC_INPUT_VAL) * 2.0f - 1.0f) * rollLimitRate;
    float pitchRateSetpoint = ((controlInputs.pitch / MAX_RC_INPUT_VAL) * 2.0f - 1.0f) * pitchLimitRate;
    float yawRateSetpoint = ((controlInputs.yaw / MAX_RC_INPUT_VAL) * 2.0f - 1.0f) * yawLimitRate;

    float rollRateMeasured = droneState.rollRate;
    float pitchRateMeasured = droneState.pitchRate;
    float yawRateMeasured = droneState.yawRate;

    // Run PID, outputs control effort in [-1,1]
    controlInputs.roll = rollPID.pidOutput(rollRateSetpoint, rollRateMeasured);
    controlInputs.pitch = pitchPID.pidOutput(pitchRateSetpoint, pitchRateMeasured);
    controlInputs.yaw = yawPID.pidOutput(yawRateSetpoint, yawRateMeasured);

    controlInputs.throttle /= 100.0f; // Throttle remains in [0, 1]

    return controlInputs;
}
