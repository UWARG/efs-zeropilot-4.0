#include "althold_mapping.hpp"
#include <cmath>

AltholdMapping::AltholdMapping(float controlPeriodS, StabilizeMapping &stabilize) noexcept : 
    posLoopRatio(controlPeriodS / POS_LOOP_RATE_HZ),
    velLoopRatio(controlPeriodS / VEL_LOOP_RATE_HZ),
    accelLoopRatio(controlPeriodS / ACCEL_LOOP_RATE_HZ),
    positionPID(0.0f, 0.0f, 0.0f, 0.0f,
                OUTPUT_MIN, OUTPUT_MAX, 100,
                1.0f / POS_LOOP_RATE_HZ),
    velocityPID(0.0f, 0.0f, 0.0f, 0.0f,
                OUTPUT_MIN, OUTPUT_MAX, 100,
                1.0f / VEL_LOOP_RATE_HZ),
    accelPID(0.0f, 0.0f, 0.0f, 0.0f,
                OUTPUT_MIN, OUTPUT_MAX, 100,
                1.0f / ACCEL_LOOP_RATE_HZ),
    stabilizeCLAW(stabilize),
    controlPeriodS(controlPeriodS) {
        positionPID.pidInitState();
        velocityPID.pidInitState();
        accelPID.pidInitState();
}

void AltholdMapping::activateFlightMode() {
    resetControlLoopState();
    stabilizeCLAW.resetControlLoopState();
}

void AltholdMapping::resetControlLoopState() noexcept {
    positionPID.pidInitState();
    velocityPID.pidInitState();
    accelPID.pidInitState();
}

void AltholdMapping::setMaxClimbRate(float newMaxClimbRate) noexcept {
    maxClimbRate = newMaxClimbRate;
}

void AltholdMapping::setMaxDownRate(float newMaxDescendRate) noexcept {
    maxDescendRate = newMaxDescendRate;
}

RCMotorControlMessage_t AltholdMapping::runControl(RCMotorControlMessage_t controlInput, const DroneState_t &droneState) {
    // Check throttle input every loop to determine the altitude to hold
    float throttle = controlInput.throttle / 100.0f; // Convert throttle from [0, 100] to [0, 1]
    if (fabsf(throttle - 0.5f) < THROTTLE_DEADZONE) {
        // In the deadzone, maintain the current altitude
        if (!wasInDeadzone) {
            // Only set the target once when entering the deadzone
            targetAlt = droneState.altitude;
            wasInDeadzone = true;
        }
    } else {
        // Outside the deadzone, adjust the target altitude based on throttle input
        if (throttle > 0.5f) { // Climb
            targetAlt += throttle * maxClimbRate * controlPeriodS;
        } else  { // Descend
            targetAlt += throttle * maxDescendRate * controlPeriodS;
        }
        wasInDeadzone = false;
    }

    // Position control loop
    if (posLoopCounter == 0) {
        float posSetpoint = targetAlt;
        float posMeasrued = droneState.altitude;
        velocityCmd = positionPID.pidOutput(posSetpoint, posMeasrued);
    }

    // Velocity control loop
    if (velLoopCounter == 0) {
        float velSetpoint = velocityCmd;
        float velMeasured = droneState.verticleVel;
        accelCmd = velocityPID.pidOutput(velSetpoint, velMeasured);
    }

    // Acceleration control loop
    if (accelLoopCounter == 0) {
        float accelSetpoint = accelCmd;
        float accelMeasured = droneState.verticleAcc;
        controlInput.throttle = accelPID.pidOutput(accelSetpoint, accelMeasured);
    }

    posLoopCounter = (posLoopCounter + 1) % posLoopRatio;
    velLoopCounter = (velLoopCounter + 1) % velLoopRatio;
    accelLoopCounter = (accelLoopCounter + 1) % accelLoopRatio;

    // Stabilize runs seperately every loop
    controlInputs = stabilizeCLAW.runControl(controlInput, droneState);

    return controlInputs;
}

