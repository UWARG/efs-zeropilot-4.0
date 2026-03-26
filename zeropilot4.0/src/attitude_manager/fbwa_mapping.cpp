#include "fbwa_mapping.hpp"
#include "error.h"

FBWAMapping::FBWAMapping(float control_iter_period_s) noexcept :
    rollPID(0.0f, 0.0f, 0.0f,
        0.0f, OUTPUT_MIN, OUTPUT_MAX,
        INTEGRAL_MIN, INTEGRAL_MAX, control_iter_period_s),
    pitchPID(0.0f, 0.0f, 0.0f,
        0.0f, OUTPUT_MIN, OUTPUT_MAX,
        INTEGRAL_MIN, INTEGRAL_MAX, control_iter_period_s),
    yawRudderMixingConst(0.0f)
{
    rollPID.pidInitState();
    pitchPID.pidInitState();
}

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

// Resetter for both roll and pitch PIDs
ZP_ERROR_e FBWAMapping::resetControlLoopState() noexcept {
    ZP_RETURN_IF_ERROR(rollPID.pidInitState());
    ZP_RETURN_IF_ERROR(pitchPID.pidInitState());
    return ZP_ERROR_OK;
}

// Setter for *yaw* rudder mixing const
ZP_ERROR_e FBWAMapping::setYawRudderMixingConstant(float newMixingConst) noexcept {
    yawRudderMixingConst = newMixingConst;
    return ZP_ERROR_OK;
}

ZP_ERROR_e FBWAMapping::activateFlightMode() {
    ZP_RETURN_IF_ERROR(resetControlLoopState());
    return ZP_ERROR_OK;
}

// Main control mapping function for FBWA mode
ZP_ERROR_e FBWAMapping::runControl(RCMotorControlMessage_t *controlOutput, RCMotorControlMessage_t controlInputs, const DroneState_t &droneState){
    if (controlOutput == nullptr) {
        return ZP_ERROR_NULLPTR;
    }

    // Convert RC inputs into radians
    float rollSetpoint = (controlInputs.roll / MAX_RC_INPUT_VAL) * (ROLL_MAX_ANGLE_RAD - ROLL_MIN_ANGLE_RAD) + ROLL_MIN_ANGLE_RAD;
    float pitchSetpoint = (controlInputs.pitch / MAX_RC_INPUT_VAL) * (PITCH_MAX_ANGLE_RAD - PITCH_MIN_ANGLE_RAD) + PITCH_MIN_ANGLE_RAD;

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

    *(controlOutput) = controlInputs;

    return ZP_ERROR_OK;
}
