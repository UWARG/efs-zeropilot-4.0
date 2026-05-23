#include "fbwb_mapping.hpp"

FBWBMapping::FBWBMapping(float control_iter_period_s) noexcept :
    FBWAMapping(control_iter_period_s), // all FBWA fields are already initialized
    totalEnergyPID(0.0f, 0.0f, 0.0f,
        0.0f, PID_OUTPUT_MIN, PID_OUTPUT_MAX, 100,
        control_iter_period_s * OUTER_LOOP_DIVIDER),
    energyBalancePID(0.0f, 0.0f, 0.0f,
        0.0f, PID_OUTPUT_MIN, PID_OUTPUT_MAX, 100,
        control_iter_period_s * OUTER_LOOP_DIVIDER),
    isInitialized(false),
    targetAltitude_m(0.0f),
    currentPitchSetpoint(0.0f),
    currentThrottleOutput_pct(50.0f),
    outerLoopSchedulingCounter(0),
    dt_s(control_iter_period_s)
{
    totalEnergyPID.pidInitState();
    energyBalancePID.pidInitState();
}

void FBWBMapping::setTotalEnergyPIDConstants(float newKp, float newKi, float newKd, float newTau, uint8_t newIMaxPct) noexcept {
    totalEnergyPID.setConstants(newKp, newKi, newKd, newTau, newIMaxPct);
}

void FBWBMapping::setEnergyBalancePIDConstants(float newKp, float newKi, float newKd, float newTau, uint8_t newIMaxPct) noexcept {
    energyBalancePID.setConstants(newKp, newKi, newKd, newTau, newIMaxPct);
}

void FBWBMapping::resetControlLoopState() noexcept {
    FBWAMapping::resetControlLoopState();
    totalEnergyPID.pidInitState();
    energyBalancePID.pidInitState();
}

void FBWBMapping::activateFlightMode() {
    FBWAMapping::activateFlightMode();
    resetControlLoopState();
    isInitialized = false;
    outerLoopSchedulingCounter = 0;
}

RCMotorControlMessage_t FBWBMapping::runControl(RCMotorControlMessage_t controlInputs, const DroneState_t &droneState) {
    if(!isInitialized){
        targetAltitude_m = droneState.altitude;
        targetAirspeed_mps = droneState.airspeed;
        currentPitchSetpoint = droneState.pitch;
        isInitialized = true;
    }
    
    // because the target altitude is an integration of pitch, we add this each time
    // TODO: piecewise scaling [0, 50], [50, 100] Look at FBWA pitch angle logic
    float targetClimbRate = (controlInputs.pitch / MAX_RC_INPUT_VAL) * (MAX_ALTITUDE_DELTA_MPS - MIN_ALTITUDE_DELTA_MPS) + MIN_ALTITUDE_DELTA_MPS; // convert to m/s
    targetAltitude_m += targetClimbRate * dt_s; // integration

    // run every 10 calls, and run on initialization too. The counter loops from 0 - 9.
    if(outerLoopSchedulingCounter == 0){
        // convert to m/s and scale to [20, 60]
        float targetThrottle = (controlInputs.throttle / MAX_RC_INPUT_VAL) * (MAX_AIRSPEED_MPS - MIN_AIRSPEED_MPS) + MIN_AIRSPEED_MPS;
        targetAirspeed_mps = targetThrottle;

        float height = targetAltitude_m;
        float velocity = targetThrottle;

        float measuredVelocity = droneState.airspeed;
        float measuredHeight = droneState.altitude;
        
        float totalEnergy = (velocity * velocity) / (2 * GRAVITY_MSS) + height;
        float energyBalance = height - (velocity * velocity) / (2 * GRAVITY_MSS);

        float meauredTotalEnergy = (measuredVelocity * measuredVelocity) / (2 * GRAVITY_MSS) + measuredHeight;
        float measuredEnergyBalance = measuredHeight - (measuredVelocity * measuredVelocity) / (2 * GRAVITY_MSS);

        float throttleOutput = totalEnergyPID.pidOutput(totalEnergy, meauredTotalEnergy);
        float pitchOutput = energyBalancePID.pidOutput(energyBalance, measuredEnergyBalance);

        currentThrottleOutput_pct = (throttleOutput * PID_OUTPUT_SCALE) + PID_OUTPUT_SHIFT;
        currentPitchSetpoint = (pitchOutput * PID_OUTPUT_SCALE) + PID_OUTPUT_SHIFT;
    }
    outerLoopSchedulingCounter = (++outerLoopSchedulingCounter) % OUTER_LOOP_DIVIDER;

    controlInputs.pitch = currentPitchSetpoint;
    controlInputs.throttle = currentThrottleOutput_pct;

    RCMotorControlMessage_t innerLoopControlOutputs = FBWAMapping::runControl(controlInputs, droneState); // call the inner loop on the active frequency, should be 100Hz

    return innerLoopControlOutputs;
}

float FBWBMapping::calculateSpecificKE(float airspeed_mps) {
    return (airspeed_mps * airspeed_mps) / (2.0f * GRAVITY_MSS);
}
