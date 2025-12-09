#include "fbwa_mapping.hpp"

FBWAMapping::FBWAMapping() noexcept :
    rollPID(0.0f, 0.0f, 0.0f,
        0.0f, OUTPUT_MIN, OUTPUT_MAX,
        ROLL_INTEGRAL_MIN_LIM, ROLL_INTEGRAL_MAX_LIM, AM_CONTROL_LOOP_DELAY),
    pitchPID(0.0f, 0.0f, 0.0f,
        0.0f, OUTPUT_MIN, OUTPUT_MAX,
        PITCH_INTEGRAL_MIN_LIM, PITCH_INTEGRAL_MAX_LIM, AM_CONTROL_LOOP_DELAY)
{
    rollPID.pidInitState();
    pitchPID.pidInitState();
}

FBWAMapping::~FBWAMapping() noexcept {}

// Setter *roll* for PID consts
void FBWAMapping::setRollPIDConstants(float newKp, float newKi, float newKd, float newTau) noexcept {
    rollPID.setConstants(newKp, newKi, newKd, newTau);
}

// Setter for *pitch* PID consts
void FBWAMapping::setPitchPIDConstants(float newKp, float newKi, float newKd, float newTau) noexcept {
    pitchPID.setConstants(newKp, newKi, newKd, newTau);
}

RCMotorControlMessage_t FBWAMapping::runControl(RCMotorControlMessage_t controlInputs){
    float rollSetpoint = controlInputs.roll;
    float pitchSetpoint = controlInputs.pitch;

    // Get from sensor fusion -- make sure to update include files when we get this ********
    float rollMeasured = 50.0;
    float pitchMeasured = 50.0;

    // Currently, roll & pitch outputs receive absolute roll & pitch angles, not relative to current position.
    float rollOutput = rollPID.pidOutput(rollSetpoint, rollMeasured);
    float pitchOutput = pitchPID.pidOutput(pitchSetpoint, pitchMeasured);

    controlInputs.roll = rollOutput; // setting desired roll angle
    controlInputs.pitch = pitchOutput; // setting desired pitch angle

    return controlInputs;
}