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
void ACROMapping::setRollPIDConstants(float newKp, float newKi, float newKd, float newTau, uint8_t newIMaxPct) noexcept
{
    rollPID.setConstants(newKp, newKi, newKd, newTau, newIMaxPct);
}

// Setter for *pitch* PID consts
void ACROMapping::setPitchPIDConstants(float newKp, float newKi, float newKd, float newTau, uint8_t newIMaxPct) noexcept
{
    pitchPID.setConstants(newKp, newKi, newKd, newTau, newIMaxPct);
}

// Resetter for both roll and pitch PIDs (needed for unit testing)
void ACROMapping::resetControlLoopState() noexcept
{
    rollPID.pidInitState();
    pitchPID.pidInitState();
}

// Setter for *rollLimitAngle* in rad
void setRollLimitAngle(float newRollLimitAngle) noexcept
{
    rollLimitAngle = newRollLimitAngle;
}

// Setter for *pitchLimitAngle* in rad
void setPitchLimitAngle(float newPitchLimitAngle) noexcept
{
    pitchLimitAngle = newPitchLimitAngle;
}

// Getter for PID objects
PID *ACROMapping::getRollPID() noexcept { return &rollPID; }
PID *ACROMapping::getPitchPID() noexcept { return &pitchPID; }

void ACROMapping::activateFlightMode()
{
    resetControlLoopState();
}

// Main control mapping function for ACRO mode
RCMotorControlMessage_t ACROMapping::runControl(RCMotorControlMessage_t controlInputs, const DroneState_t &droneState)
{
    // Setpoints: Maps [0, 100] to [-limit, +limit]
    float rollAngleSetpoint = ((controlInputs.roll / MAX_RC_INPUT_VAL) * 2.0f - 1.0f) * rollLimitAngle;
    float pitchAngleSetpoint = ((controlInputs.pitch / MAX_RC_INPUT_VAL) * 2.0f - 1.0f) * pitchLimitAngle;

    float rollAngleMeasured = droneState.roll;
    float pitchAngleMeasured = droneState.pitch;

    // Run PID, outputs control effort in [-1,1]
    controlInputs.roll = rollPID.pidOutput(rollAngleSetpoint, rollAngleMeasured);
    controlInputs.pitch = pitchPID.pidOutput(pitchAngleSetpoint, pitchAngleMeasured);

    // scale control efforts back to RC controller range [0,100] for acro control loop
    

    
    // Run acro control
    acroCLAW.runControl(controlInputs, droneState);

    return controlInputs;
}


const float *ACROMapping::getMixedMotors() {
    return motorPercent;
}