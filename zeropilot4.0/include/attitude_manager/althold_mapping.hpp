#pragma once

#include "flightmode.hpp"
#include "stabilize_mapping.hpp"
#include "pid.hpp"

class AltholdMapping : public Flightmode {
public:
    AltholdMapping(float controlPeriodS, StabilizeMapping &stabilize) noexcept;

    void activateFlightMode() override;

    RCMotorControlMessage_t runControl(RCMotorControlMessage_t controlInput, const DroneState_t &droneState) override;

    void updateTerrainAlt(const DroneState_t &droneState, float rangefinderAlt);

    // Resetter for all roll, pitch and yaw PIDs (needed for unit testing)
    void resetControlLoopState() noexcept;

    /*
    Setters
    */
    void setPositionPIDConstants(float newKp, float newKi, float newKd, float newTau, uint8_t newIMaxPct) noexcept;
    void setVelocityPIDConstants(float newKp, float newKi, float newKd, float newTau, uint8_t newIMaxPct) noexcept;
    void setAccelPIDConstants(float newKp, float newKi, float newKd, float newTau, uint8_t newIMaxPct) noexcept;

    void setMaxClimbRate(float newMaxClimbRate) noexcept;
    void setMaxDescendRate(float newMaxDescendRate) noexcept;
    void setPilotAccelRate(float newPilotAccelRate) noexcept;
    void setHoverThrottle(float newHoverThrottle) noexcept;

    // Getter for PID objects
    PID *getPosPID() noexcept;
    PID *getVelPID() noexcept;
    PID *getAccelPID() noexcept;

    // Destructor
    ~AltholdMapping() noexcept override = default;

private:
    static constexpr uint32_t POS_LOOP_RATE_HZ = 25;
    static constexpr uint32_t VEL_LOOP_RATE_HZ = 100;
    static constexpr uint32_t ACCEL_LOOP_RATE_HZ = 200;

    uint32_t posLoopRatio;
    uint32_t velLoopRatio;
    uint32_t accelLoopRatio;
    uint32_t rangefinderTimeoutCycles;

    uint8_t posLoopCounter = 0;
    uint8_t velLoopCounter = 0;
    uint8_t accelLoopCounter = 0;

    // Output limits (for control effort)
    static constexpr float OUTPUT_MIN = -1.0f;
    static constexpr float OUTPUT_MAX = +1.0f;

    PID positionPID; // Position loop should have only P term
    PID velocityPID; // Velocity/rate loop should have P and D terms
    PID accelPID; // Acceleration loop should have all PID terms

    StabilizeMapping &stabilizeCLAW;

    float controlPeriodS;

    bool needTargetAltInit = true;
    float targetAlt = 0.0f;
    float targetAltAboveTerrain = 0.0f;
    float terrainAlt = 0.0f;
    bool rangefinderLost = true;
    uint32_t rangefinderStaleCycles = 0;
    uint8_t validReadingCount = 0;

    static constexpr float RANGEFINDER_TIMEOUT_S = 0.3f;
    static constexpr uint8_t RANGEFINDER_REACQUIRE_COUNT = 5;

    float maxClimbRate = 2.5f;
    float maxDescendRate = 2.5f;
    float pilotAccelRate = 2.5f; // How aggressively can the pilot's stick input change the climb rate
    float maxRateChange = 0.0f;
    float lastDesiredRate = 0.0f;
    
    // Constrain throttle to not span the entire range to reserve space for motor mixing
    static constexpr float minThrottle = 0.1f;
    static constexpr float maxThrottle = 0.9f;
    
    static constexpr float THROTTLE_DEADZONE = 0.1f;
    bool wasInDeadzone = true;

    float hoverThrottle = 0.3f;

    static constexpr float ALT_LEASH = 5.0f;

    float velocityCmd = 0.0f;
    float accelCmd = 0.0f;
    float throttleCmd = 0.0f;
    
    void updateHoverThrottle(float currentThrottle, float verticalVel, float verticalAcc, float altitude);
};