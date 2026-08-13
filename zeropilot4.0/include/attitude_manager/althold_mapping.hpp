#pragma once

#include "flightmode.hpp"
#include "stabilize_mapping.hpp"
#include "pid.hpp"

class AltholdMapping : public Flightmode {
public:
    AltholdMapping(float controlPeriodS, StabilizeMapping &stabilize) noexcept;

    void activateFlightMode(const DroneState_t &droneState) override;

    RCMotorControlMessage_t runControl(RCMotorControlMessage_t controlInput, const DroneState_t &droneState) override;

    /*
    Setters
    */

    void setMaxClimbRate(float newMaxClimbRate) noexcept;
    void setMaxDownRate(float newMaxDescendRate) noexcept;
    void setHoverThrottle(float newHoverThrottle) noexcept;

    // Resetter for all roll, pitch and yaw PIDs (needed for unit testing)
    void resetControlLoopState() noexcept;

    // Getter for PID objects

    // Destructor
    ~AltholdMapping() noexcept override = default;

private:
    static constexpr uint32_t POS_LOOP_RATE_HZ = 25;
    static constexpr uint32_t VEL_LOOP_RATE_HZ = 100;
    static constexpr uint32_t ACCEL_LOOP_RATE_HZ = 200;

    uint32_t posLoopRatio;
    uint32_t velLoopRatio;
    uint32_t accelLoopRatio;
    
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

    float targetAlt;
    
    float velocityCmd;
    float accelCmd;

    float maxClimbRate = 2.5f;
    float maxDescendRate = 2.5f;
    
    // Constrain throttle to not span the entire range to reserve space for motor mixing
    static constexpr float minThrottle = 0.1f;
    static constexpr float maxThrottle = 0.9f;
    
    static constexpr float THROTTLE_DEADZONE = 0.1f;
    bool wasInDeadzone = true;

    float hoverThrottle = 0.3f;

    static constexpr float ALT_LEASH = 5.0f;
    
    void updateHoverThrottle(float currentThrottle, float verticalVel, float verticalAcc);
};