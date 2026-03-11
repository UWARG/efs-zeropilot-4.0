#include "fbwa_mapping.hpp"

#define FBWA_PID_OUTPUT_SHIFT 50.0f

FBWAMapping::FBWAMapping() :
    rollPID(0.0f, 0.0f, 0.0f,
        0.0f, OUTPUT_MIN, OUTPUT_MAX,
        ROLL_INTEGRAL_MIN_LIM, ROLL_INTEGRAL_MAX_LIM, AM_CONTROL_LOOP_PERIOD_S),
    pitchPID(0.0f, 0.0f, 0.0f,
        0.0f, OUTPUT_MIN, OUTPUT_MAX,
        PITCH_INTEGRAL_MIN_LIM, PITCH_INTEGRAL_MAX_LIM, AM_CONTROL_LOOP_PERIOD_S)
{}

ZP_ERROR_e FBWAMapping::init() {
    if (isInitialized) return ZP_ERROR_ALREADY_INITIALIZED;
    ZP_RETURN_IF_ERROR(rollPID.pidInitState());
    ZP_RETURN_IF_ERROR(pitchPID.pidInitState());
    isInitialized = true;
    return ZP_ERROR_OK;
}

FBWAMapping::~FBWAMapping() noexcept {}

// Setter *roll* for PID consts
ZP_ERROR_e FBWAMapping::setRollPIDConstants(float newKp, float newKi, float newKd, float newTau) noexcept {
    ZP_RETURN_IF_ERROR(rollPID.setConstants(newKp, newKi, newKd, newTau));
    return ZP_ERROR_OK;
}

// Setter for *pitch* PID consts
ZP_ERROR_e FBWAMapping::setPitchPIDConstants(float newKp, float newKi, float newKd, float newTau) noexcept {
    ZP_RETURN_IF_ERROR(pitchPID.setConstants(newKp, newKi, newKd, newTau));
    return ZP_ERROR_OK;
}

ZP_ERROR_e FBWAMapping::runControl(RCMotorControlMessage_t *controlOutput, RCMotorControlMessage_t controlInputs, const DroneState_t &droneState){
    if (control == nullptr) {
        return ZP_ERROR_NULLPTR;
    }

    // Convert RC inputs into radians
    float rollSetpoint = (controlInputs.roll / (float)MAX_RC_INPUT_VAL) * (ROLL_MAX_ANGLE_RAD - ROLL_MIN_ANGLE_RAD) + ROLL_MIN_ANGLE_RAD;
    float pitchSetpoint = (controlInputs.pitch / (float)MAX_RC_INPUT_VAL) * (PITCH_MAX_ANGLE_RAD - PITCH_MIN_ANGLE_RAD) + PITCH_MIN_ANGLE_RAD;

    // Get measured values from drone state (populated by IMU)
    float rollMeasured = droneState.roll;
    float pitchMeasured = droneState.pitch;

    // Currently, roll & pitch outputs receive absolute roll & pitch angles, not relative to current position.
    float rollOutput = rollPID.pidOutput(rollSetpoint, rollMeasured);
    float pitchOutput = pitchPID.pidOutput(pitchSetpoint, pitchMeasured);

    controlInputs.roll = rollOutput + FBWA_PID_OUTPUT_SHIFT; // setting desired roll angle, adding 50 to shift to [0,100] range
    controlInputs.pitch = pitchOutput + FBWA_PID_OUTPUT_SHIFT; // setting desired pitch angle, adding 50 to shift to [0,100] range

    *(control) = controlInputs;

    return ZP_ERROR_OK;
}
