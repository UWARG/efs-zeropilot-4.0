#include "fbwa_mapping.hpp"

#define FBWA_PID_OUTPUT_SHIFT 50.0f

FBWAMapping::FBWAMapping(float control_iter_period_s) noexcept :
    rollPID(0.0f, 0.0f, 0.0f,
        0.0f, OUTPUT_MIN, OUTPUT_MAX,
        ROLL_INTEGRAL_MIN_LIM, ROLL_INTEGRAL_MAX_LIM, control_iter_period_s),
    pitchPID(0.0f, 0.0f, 0.0f,
        0.0f, OUTPUT_MIN, OUTPUT_MAX,
        PITCH_INTEGRAL_MIN_LIM, PITCH_INTEGRAL_MAX_LIM, control_iter_period_s)
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

RCMotorControlMessage_t FBWAMapping::runControl(RCMotorControlMessage_t controlInputs, const DroneState_t &droneState){
    // Convert RC inputs into radians
    float rollSetpoint = (controlInputs.roll / MAX_RC_INPUT_VAL) * (ROLL_MAX_ANGLE_RAD - ROLL_MIN_ANGLE_RAD) + ROLL_MIN_ANGLE_RAD;
    float pitchSetpoint = (controlInputs.pitch / MAX_RC_INPUT_VAL) * (PITCH_MAX_ANGLE_RAD - PITCH_MIN_ANGLE_RAD) + PITCH_MIN_ANGLE_RAD;

    // Get measured values from drone state (populated by IMU)
    float rollMeasured = droneState.roll;
    float pitchMeasured = droneState.pitch;

    // Currently, roll & pitch outputs receive absolute roll & pitch angles, not relative to current position.
    float rollOutput = rollPID.pidOutput(rollSetpoint, rollMeasured);
    float pitchOutput = pitchPID.pidOutput(pitchSetpoint, pitchMeasured);

    controlInputs.roll = rollOutput + FBWA_PID_OUTPUT_SHIFT; // setting desired roll angle, adding 50 to shift to [0,100] range
    controlInputs.pitch = pitchOutput + FBWA_PID_OUTPUT_SHIFT; // setting desired pitch angle, adding 50 to shift to [0,100] range

    return controlInputs;
}
