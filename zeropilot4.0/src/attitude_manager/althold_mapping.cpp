#include "althold_mapping.hpp"
#include <cmath>

static float constrain(float value, float minVal, float maxVal);

AltholdMapping::AltholdMapping(float controlPeriodS, StabilizeMapping &stabilize) noexcept : 
    posLoopRatio((1.0f / controlPeriodS) / POS_LOOP_RATE_HZ),
    velLoopRatio((1.0f / controlPeriodS) / VEL_LOOP_RATE_HZ),
    accelLoopRatio((1.0f / controlPeriodS) / ACCEL_LOOP_RATE_HZ),
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

void AltholdMapping::activateFlightMode(const DroneState_t &droneState) {
    targetAlt = droneState.altitude;
    wasInDeadzone = true;  // Start in hold mode, not climbing
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

void AltholdMapping::setHoverThrottle(float newHoverThrottle) noexcept {
    hoverThrottle = newHoverThrottle;
}

RCMotorControlMessage_t AltholdMapping::runControl(RCMotorControlMessage_t controlInput, const DroneState_t &droneState) {
    // Check throttle input every loop to determine the altitude to hold
    float throttleCentered = (controlInput.throttle / 100.0f) - 0.5f; // Convert throttle from [0, 100] to [-0.5, 0.5]
    if (fabsf(throttleCentered) < THROTTLE_DEADZONE) {
        // In the deadzone, maintain the current altitude
        if (!wasInDeadzone) {
            // Only set the target once when entering the deadzone
            targetAlt = droneState.altitude;
            wasInDeadzone = true;
        }
    } else {
        // Outside the deadzone, adjust the target altitude based on throttle input
        if (throttleCentered > 0) { // [0, 0.5], climb
            targetAlt += (throttleCentered * 2.0f) * maxClimbRate * controlPeriodS; // *2.0f to scale to [0, 1] range
        } else  { // [-0.5, 0], Descend
            targetAlt += (throttleCentered * 2.0f) * maxDescendRate * controlPeriodS; // *2.0f to scale to [-1, 0] range
        }
        wasInDeadzone = false;
    }

    // Leash the target altitude to avoid too much thrust (eg. when there is gust the drone may be not 
    // climbing but the pilot may keep the throttle high, then target altitude will be really high 
    // and when the gust is gone the drone will rocket up)
    targetAlt = constrain(targetAlt, droneState.altitude - ALT_LEASH, droneState.altitude + ALT_LEASH);

    // Position control loop
    if (posLoopCounter == 0) {
        float posSetpoint = targetAlt;
        float posMeasrued = droneState.altitude;
        velocityCmd = positionPID.pidOutput(posSetpoint, posMeasrued);
    }

    // Velocity control loop
    if (velLoopCounter == 0) {
        float velSetpoint = velocityCmd;
        float velMeasured = droneState.verticalVel;
        accelCmd = velocityPID.pidOutput(velSetpoint, velMeasured);
    }

    // Acceleration control loop
    if (accelLoopCounter == 0) {
        float accelSetpoint = accelCmd;
        float accelMeasured = droneState.verticalAcc;
        float throttleCmd = hoverThrottle + accelPID.pidOutput(accelSetpoint, accelMeasured);

        // Update hover throttle based on current throttle (before tilt compansation as we want the leveled throttle)
        updateHoverThrottle(throttleCmd, droneState.verticalVel, droneState.verticalAcc);
        
        // Tilt compensation (thrustVertical = thrustTotal * cos(roll) * cos(pitch))
        // We can only command total thrust so we convert the required vertical thrust to total thrust
        throttleCmd /= (cosf(droneState.roll) * cosf(droneState.pitch));
        
        // Convert back to [0, 100] range
        controlInput.throttle = constrain(throttleCmd, minThrottle, maxThrottle) * 100.0f;
    }

    posLoopCounter = (posLoopCounter + 1) % posLoopRatio;
    velLoopCounter = (velLoopCounter + 1) % velLoopRatio;
    accelLoopCounter = (accelLoopCounter + 1) % accelLoopRatio;

    // Stabilize runs seperately every loop
    controlInput = stabilizeCLAW.runControl(controlInput, droneState);

    return controlInput;
}

void AltholdMapping::updateHoverThrottle(float currentThrottle, float verticalVel, float verticalAcc) {
    bool stable = (fabsf(verticalVel) < 0.5f) && (fabsf(verticalAcc) < 1.0f);
    if (stable) {
        // First order low pass filter to learn the hover throttle over time
        const float timeConstant = 5.0f; // 5 seconds
        const float alpha = (controlPeriodS * accelLoopRatio) / timeConstant;
        hoverThrottle += alpha * (currentThrottle - hoverThrottle);
    }
}

static float constrain(float value, float minVal, float maxVal) {
    if (value < minVal) {
        return minVal;
    } else if (value > maxVal) {
        return maxVal;
    } else {
        return value;
    }
}
