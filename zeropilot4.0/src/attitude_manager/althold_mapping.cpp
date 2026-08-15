#include "althold_mapping.hpp"
#include <cmath>

static float constrain(float value, float minVal, float maxVal);
static float scale(float value, float minVal, float maxVal);

AltholdMapping::AltholdMapping(float controlPeriodS, StabilizeMapping &stabilize) noexcept : 
    posLoopRatio((1.0f / controlPeriodS) / POS_LOOP_RATE_HZ),
    velLoopRatio((1.0f / controlPeriodS) / VEL_LOOP_RATE_HZ),
    accelLoopRatio((1.0f / controlPeriodS) / ACCEL_LOOP_RATE_HZ),
    rangefinderTimeoutCycles(static_cast<uint32_t>(RANGEFINDER_TIMEOUT_S / controlPeriodS)),
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
    controlPeriodS(controlPeriodS),
    terrainAlpha(controlPeriodS / TERRAIN_TAU_S) {
        positionPID.pidInitState();
        velocityPID.pidInitState();
        accelPID.pidInitState();
}

void AltholdMapping::activateFlightMode() {
    needTargetAltInit = true; // Initialize target altitude on first runControl loop
    wasInDeadzone = true; // Start in hold mode, not climbing

    // Initialize the surface tracking states
    targetAltAboveTerrain = 0.0f;
    terrainAlt = 0.0f; // terrainAlt starts at 0 so the drone holds an absolute altitude until the rangefinder is acquired
    rangefinderLost = true; // Assume no rangefinder, updateTerrainAlt() will update this if we have one
    rangefinderStaleCycles = 0;
    validReadingCount = 0;

    resetControlLoopState();
    stabilizeCLAW.resetControlLoopState();
}

void AltholdMapping::resetControlLoopState() noexcept {
    positionPID.pidInitState();
    velocityPID.pidInitState();
    accelPID.pidInitState();

    posLoopCounter = 0;
    velLoopCounter = 0;
    accelLoopCounter = 0;

    velocityCmd = 0.0f;
    accelCmd = 0.0f;
    throttleCmd = hoverThrottle * 100.0f; // Convert to [0, 100] range

    lastDesiredRate = 0.0f;
}

void AltholdMapping::setPositionPIDConstants(float newKp, float newKi, float newKd, float newTau, uint8_t newIMaxPct) noexcept {
    positionPID.setConstants(newKp, newKi, newKd, newTau, newIMaxPct);
}

void AltholdMapping::setVelocityPIDConstants(float newKp, float newKi, float newKd, float newTau, uint8_t newIMaxPct) noexcept {
    velocityPID.setConstants(newKp, newKi, newKd, newTau, newIMaxPct);
}

void AltholdMapping::setAccelPIDConstants(float newKp, float newKi, float newKd, float newTau, uint8_t newIMaxPct) noexcept {
    accelPID.setConstants(newKp, newKi, newKd, newTau, newIMaxPct);
}

void AltholdMapping::setMaxClimbRate(float newMaxClimbRate) noexcept {
    maxClimbRate = newMaxClimbRate;
}

void AltholdMapping::setMaxDescendRate(float newMaxDescendRate) noexcept {
    maxDescendRate = newMaxDescendRate;
}

void AltholdMapping::setPilotAccelRate(float newPilotAccelRate) noexcept {
    pilotAccelRate = newPilotAccelRate;
    maxRateChange = pilotAccelRate * controlPeriodS;
}

void AltholdMapping::setHoverThrottle(float newHoverThrottle) noexcept {
    hoverThrottle = newHoverThrottle;
}

PID *AltholdMapping::getPosPID() noexcept {
    return &positionPID;
}

PID *AltholdMapping::getVelPID() noexcept {
    return &velocityPID;
}

PID *AltholdMapping::getAccelPID() noexcept {
    return &accelPID;
}

RCMotorControlMessage_t AltholdMapping::runControl(RCMotorControlMessage_t controlInput, const DroneState_t &droneState) {
    if (needTargetAltInit) {
        targetAltAboveTerrain = droneState.altitude - terrainAlt;
        needTargetAltInit = false;
    }

    // Low pass filter the terrain altitude
    terrainAlt += terrainAlpha * (rawTerrainAlt - terrainAlt);

    // Check throttle input every loop to determine the altitude to hold
    float throttleCentered = (controlInput.throttle / 100.0f) - 0.5f; // Convert throttle from [0, 100] to [-0.5, 0.5]
    if (fabsf(throttleCentered) < THROTTLE_DEADZONE) {
        // In the deadzone, maintain the current altitude
        if (!wasInDeadzone) {
            // Only set the target once when entering the deadzone
            float vz = droneState.verticalVel;
            float stoppingDistance = constrain((vz * fabsf(vz)) / (2.0f * pilotAccelRate), 0.0f, 1.0f); // d = v^2 / (2a)
            targetAltAboveTerrain = droneState.altitude - terrainAlt + stoppingDistance;
            lastDesiredRate = 0.0f;
            wasInDeadzone = true;
        }
    } else {
        // Outside the deadzone, adjust the target altitude based on throttle input

        // Scale the stick input to [0,1] past the deadzone so rates start at 0 from the deadzone edge instead of jumping
        float stickPastDeadzone = (fabsf(throttleCentered) - THROTTLE_DEADZONE) / (0.5f - THROTTLE_DEADZONE);

        float targetRate = 0.0f;
        if (throttleCentered > 0) { // [0, 0.5], climb
            targetRate = stickPastDeadzone * maxClimbRate;
        } else  { // [-0.5, 0], Descend
            targetRate = -stickPastDeadzone * maxDescendRate;
        }
        // Limit the rate change to the pilot's desired acceleration rate
        float rateChange = constrain(targetRate - lastDesiredRate, -maxRateChange, maxRateChange);
        lastDesiredRate += rateChange; // lastDesiredRate now represents the desired rate thhis loop
        targetAltAboveTerrain += lastDesiredRate * controlPeriodS;

        // Leash the targetAltAboveTerrain to avoid too much thrust (eg. when there is gust the drone may be not
        // climbing but the pilot may keep the throttle high, then target altitude will be really high
        // and when the gust is gone the drone will rocket up)
        float altAboveTerrain = droneState.altitude - terrainAlt;
        targetAltAboveTerrain = constrain(targetAltAboveTerrain, altAboveTerrain - ALT_LEASH, altAboveTerrain + ALT_LEASH);

        wasInDeadzone = false;
    }

    // Determine if the rangefinder is lost
    if (rangefinderStaleCycles < rangefinderTimeoutCycles) {
        rangefinderStaleCycles++;
    } else {
        // Rangefinder is lost
        rangefinderLost = true;
        validReadingCount = 0;
    }

    // terrainAlt follows the terrain while the rangefinder is valid and freezes when it isn't
    // When rangefiner is lost, targetAlt simply becomes the absolute altitude
    targetAlt = targetAltAboveTerrain + terrainAlt;

    // Position control loop
    if (posLoopCounter == 0) {
        // Only ask for maximum of ALT_LEASH climb at a time so a big terrain change is taken in steps
        float posSetpoint = constrain(targetAlt, droneState.altitude - ALT_LEASH, droneState.altitude + ALT_LEASH);
        float posMeasrued = droneState.altitude;
        velocityCmd = positionPID.pidOutput(posSetpoint, posMeasrued);
        velocityCmd = scale(velocityCmd, -maxDescendRate, maxClimbRate);
    }

    // Velocity control loop
    if (velLoopCounter == 0) {
        float velSetpoint = velocityCmd;
        float velMeasured = droneState.verticalVel;
        accelCmd = velocityPID.pidOutput(velSetpoint, velMeasured);
        accelCmd = scale(accelCmd, -pilotAccelRate, pilotAccelRate);
    }

    // Acceleration control loop
    if (accelLoopCounter == 0) {
        float accelSetpoint = accelCmd;
        float accelMeasured = droneState.verticalAcc;
        float accelOutput = accelPID.pidOutput(accelSetpoint, accelMeasured);
        throttleCmd = hoverThrottle + scale(accelOutput, minThrottle - hoverThrottle, maxThrottle - hoverThrottle);

        // Update hover throttle based on current throttle (before tilt compansation since we want the leveled throttle)
        updateHoverThrottle(throttleCmd, droneState.verticalVel, droneState.verticalAcc, droneState.altitude);
        
        // Tilt compensation (thrustVertical = thrustTotal * cos(roll) * cos(pitch))
        // We can only command total thrust so we convert the required vertical thrust to total thrust
        float tiltCos = cosf(droneState.roll) * cosf(droneState.pitch);
        throttleCmd /= fmaxf(tiltCos, 0.1f); // Avoid dividing by zero if the drone is 90 deg
        
        // Convert back to [0, 100] range
        throttleCmd = constrain(throttleCmd, minThrottle, maxThrottle) * 100.0f;
    }
    controlInput.throttle = throttleCmd;

    posLoopCounter = (posLoopCounter + 1) % posLoopRatio;
    velLoopCounter = (velLoopCounter + 1) % velLoopRatio;
    accelLoopCounter = (accelLoopCounter + 1) % accelLoopRatio;

    // Stabilize runs seperately every loop
    controlInput = stabilizeCLAW.runControl(controlInput, droneState);

    return controlInput;
}

void AltholdMapping::updateHoverThrottle(float currentThrottle, float verticalVel, float verticalAcc, float altitude) {
    bool stable = (fabsf(verticalVel) < 0.5f) && (fabsf(verticalAcc) < 1.0f);
    bool inAir = altitude > 0.5f;
    if (stable && inAir) {
        // First order low pass filter to learn the hover throttle over time
        const float timeConstant = 5.0f; // 5 seconds
        const float alpha = (controlPeriodS * accelLoopRatio) / timeConstant;
        hoverThrottle += alpha * (currentThrottle - hoverThrottle);
    }
}

void AltholdMapping::updateTerrainAlt(const DroneState_t &droneState, float rangefinderAlt) {
    // Reset the stale counter since we got a new reading
    rangefinderStaleCycles = 0;

    if (needTargetAltInit) return;

    float newTerrainAlt = droneState.altitude - rangefinderAlt;

    // Attempt to reaquire the rangefinder if it was lost
    if (rangefinderLost) {
        // Require a few consecutive readings to ensure rangefinder is actually reacquired 
        // and not just a single good reading in the middle of a dropout
        validReadingCount++;
        if (validReadingCount < RANGEFINDER_REACQUIRE_COUNT) {
            return;
        }
        // The rangefinder is now considered reacquired
        validReadingCount = 0;
        
        /*
        Rebase targetAltAboveTerrain onto the new terrain so targetAlt comes out unchanged, 
        we hold the current altitude on this transition. Without this, a rangefinder that comes 
        back reading something far from the terrainAlt we were freezed to may cause the drone 
        to shoot off to correct it.
        */
        targetAltAboveTerrain = targetAlt - newTerrainAlt;
        rawTerrainAlt = newTerrainAlt;
        terrainAlt = newTerrainAlt;

        rangefinderLost = false;
    }

    rawTerrainAlt = newTerrainAlt;
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

// Map PID output in [-1, 1] onto an asymmetric range. Positive values scale by maxVal, negative values by minVal.
static float scale(float value, float minVal, float maxVal) {
    return value * (value >= 0.0f ? maxVal : -minVal);
}
