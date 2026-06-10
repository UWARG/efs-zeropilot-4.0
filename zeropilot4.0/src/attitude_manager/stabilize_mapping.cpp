#include <cmath>
#include "stabilize_mapping.hpp"
#include "unit_conversions.hpp"

STABILIZEMapping::STABILIZEMapping(float control_iter_period_s_stabilize, float control_iter_period_s_acro) noexcept :  rollPID(0.0f, 0.0f, 0.0f, 0.0f,
                                                                                                                                OUTPUT_MIN, OUTPUT_MAX, 100,
                                                                                                                                control_iter_period_s_stabilize),
                                                                                                                        pitchPID(0.0f, 0.0f, 0.0f, 0.0f,
                                                                                                                                OUTPUT_MIN, OUTPUT_MAX, 100,
                                                                                                                                control_iter_period_s_stabilize),
                                                                                                                        rollLimitAngle(0.0f),
                                                                                                                        pitchLimitAngle(0.0f),
                                                                                                                        acroCLAW(control_iter_period_s_acro)                          
{
    rollPID.pidInitState();
    pitchPID.pidInitState();
    acroCLAW.activateFlightMode();
}

// Setter *roll* for PID consts
void STABILIZEMapping::setRollPIDConstants(float newKp, float newKi, float newKd, float newTau, uint8_t newIMaxPct) noexcept
{
    rollPID.setConstants(newKp, newKi, newKd, newTau, newIMaxPct);
}

// Setter for *pitch* PID consts
void STABILIZEMapping::setPitchPIDConstants(float newKp, float newKi, float newKd, float newTau, uint8_t newIMaxPct) noexcept
{
    pitchPID.setConstants(newKp, newKi, newKd, newTau, newIMaxPct);
}

// Resetter for both roll and pitch PIDs (needed for unit testing)
void STABILIZEMapping::resetControlLoopState() noexcept
{
    rollPID.pidInitState();
    pitchPID.pidInitState();
}

// Setter for *rollLimitAngle* in rad
void STABILIZEMapping::setRollLimitAngle(float newRollLimitAngle) noexcept
{
    rollLimitAngle = newRollLimitAngle;
}

// Setter for *pitchLimitAngle* in rad
void STABILIZEMapping::setPitchLimitAngle(float newPitchLimitAngle) noexcept
{
    pitchLimitAngle = newPitchLimitAngle;
}

// Setter for ACRO *roll* for PID consts
void STABILIZEMapping::setAcroRollPIDConstants(float newKp, float newKi, float newKd, float newTau, uint8_t newIMaxPct) {
    acroCLAW.setRollPIDConstants(newKp, newKi, newKd, newTau, newIMaxPct);
}

// Setter for ACRO *pitch* PID consts
void STABILIZEMapping::setAcroPitchPIDConstants(float newKp, float newKi, float newKd, float newTau, uint8_t newIMaxPct) {
    acroCLAW.setPitchPIDConstants(newKp, newKi, newKd, newTau, newIMaxPct);
}

// Setter for ACRO *yaw* PID consts
void STABILIZEMapping::setAcroYawPIDConstants(float newKp, float newKi, float newKd, float newTau, uint8_t newIMaxPct) {
    acroCLAW.setYawPIDConstants(newKp, newKi, newKd, newTau, newIMaxPct);
}

// Setter for ACRO *rollLimitRate* in rad / s
void STABILIZEMapping::setAcroRollLimitRate(float newRollLimitRate) {
    acroCLAW.setRollLimitRate(newRollLimitRate);
}

// Setter for ACRO *pitchLimitRate* in rad / s
void STABILIZEMapping::setAcroPitchLimitRate(float newPitchLimitRate) {
    acroCLAW.setPitchLimitRate(newPitchLimitRate);
}

// Setter for ACRO *yawLimitRate* in rad / s
void STABILIZEMapping::setAcroYawLimitRate(float newYawLimitRate) {
    acroCLAW.setYawLimitRate(newYawLimitRate);
}

// Getter for PID objects
PID *STABILIZEMapping::getRollPID() noexcept { return &rollPID; }
PID *STABILIZEMapping::getPitchPID() noexcept { return &pitchPID; }

void STABILIZEMapping::activateFlightMode()
{
    resetControlLoopState();
    acroCLAW.resetControlLoopState();
}

// Main control mapping function for ACRO mode
RCMotorControlMessage_t STABILIZEMapping::runControl(RCMotorControlMessage_t controlInputs, const DroneState_t &droneState)
{
    // Setpoints: Maps [0, 100] to [-limit, +limit]
    float rollAngleSetpoint = ((controlInputs.roll / MAX_RC_INPUT_VAL) * 2.0f - 1.0f) * rollLimitAngle;
    float pitchAngleSetpoint = ((controlInputs.pitch / MAX_RC_INPUT_VAL) * 2.0f - 1.0f) * pitchLimitAngle;

    float rollAngleMeasured = droneState.roll;
    float pitchAngleMeasured = droneState.pitch;

    // Run PID, outputs control effort in [-1,1]
    controlInputs.roll = rollPID.pidOutput(rollAngleSetpoint, rollAngleMeasured);
    controlInputs.pitch = pitchPID.pidOutput(pitchAngleSetpoint, pitchAngleMeasured);

    // Scale control efforts back to RC controller range [0,100] for acro control loop
    controlInputs.roll = (controlInputs.roll + 1.0f) * 50.0f;
    controlInputs.pitch = (controlInputs.pitch + 1.0f) * 50.0f;
    
    // Run acro control
    controlInputs = acroCLAW.runControl(controlInputs, droneState);

    return controlInputs;
}