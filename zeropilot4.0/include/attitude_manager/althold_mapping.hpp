#pragma once

#include "flightmode.hpp"
#include "stabilize_mapping.hpp"
#include "pid.hpp"

class AltholdMapping : public Flightmode {
public:
    AltholdMapping(float controlPeriodS, StabilizeMapping &stabilize) noexcept;

    void activateFlightMode() override;

    RCMotorControlMessage_t runControl(RCMotorControlMessage_t controlInput, const DroneState_t &droneState) override;

    /*
    Setters
    */

    void setMaxClimbRate(float newMaxClimbRate) noexcept;
    void setMaxDownRate(float newMaxDescendRate) noexcept;

    // Resetter for all roll, pitch and yaw PIDs (needed for unit testing)
    void resetControlLoopState() noexcept;

    // Getter for PID objects

    // Destructor
    ~AltholdMapping() noexcept override = default;

private:
    uint32_t posLoopRatio;
    uint32_t velLoopRatio;
    uint32_t accelLoopRatio;
    
    PID positionPID;
    PID velocityPID;
    PID accelPID;

    StabilizeMapping &stabilizeCLAW;

    float controlPeriodS;
    
    static constexpr uint32_t POS_LOOP_RATE_HZ = 25;
    static constexpr uint32_t VEL_LOOP_RATE_HZ = 100;
    static constexpr uint32_t ACCEL_LOOP_RATE_HZ = 200;

    uint8_t posLoopCounter;
    uint8_t velLoopCounter;
    uint8_t accelLoopCounter;

    float maxClimbRate = 2.5f;
    float maxDescendRate = 2.5f;

    static constexpr float THROTTLE_DEADZONE = 0.01f;
    bool wasInDeadzone = true;

    float targetAlt;

    float velocityCmd;
    float accelCmd;
    
}