#pragma once

#include "flightmode.hpp"
#include "pid.hpp"

class FBWBMapping : public Flightmode {
    public:
        FBWBMapping(float control_iter_period_s) noexcept;

        void activateFlightMode() override;

        RCMotorControlMessage_t runControl(RCMotorControlMessage_t controlInput, const DroneState_t &droneState) override;

        void setRollPIDConstants(float newKp, float newKi, float newKd, float newTau) noexcept;
        void setPitchPIDConstants(float newKp, float newKi, float newKd, float newTau) noexcept;
        void setTotalEnergyPIDConstants(float newKp, float newKi, float newKd, float newTau) noexcept;
        void setEnergyBalancePIDConstants(float newKp, float newKi, float newKd, float newTau) noexcept;
        void setYawRudderMixingConstant(float newMixingConst) noexcept;

        void resetControlLoopState() noexcept;

        ~FBWBMapping() noexcept override = default;

    private:
        // Roll and Pitch inner PID control loops
        PID rollPID;
        PID pitchPID;

        // TECS outer PID control loops
        PID totalEnergyPID;
        PID energyBalancePID;

        // TECS State Variables
        bool isInitialized;
        float targetAltitude_m;
        float targetAirspeed_mps;
        float currentPitchSetpoint_rad;
        float currentThrottleOutput_pct;
        uint32_t outerLoopSchedulingCounter;
        float dt_s;

        // Yaw rudder mixing constant
        float yawRudderMixingConst;

        // Ratio between scheduling for inner loop and outer loop
        static constexpr uint8_t OUTER_LOOP_DIVIDER = 10;

        // Constants
        static constexpr float GRAVITY_MSS = 9.81f;

        // Output limits (for control effort)
        static constexpr float OUTPUT_MIN = -1.0f;
        static constexpr float OUTPUT_MAX = +1.0f;

        // Integral limits (to prevent windup)
        static constexpr float INTEGRAL_MIN = -0.5f;
        static constexpr float INTEGRAL_MAX = +0.5f;

        // Roll and Pitch Angle Ranges (in radians)
        static constexpr float ROLL_MIN_ANGLE_RAD = -0.785f;  // -45 deg
        static constexpr float ROLL_MAX_ANGLE_RAD = 0.785f;   // +45 deg
        static constexpr float PITCH_MIN_ANGLE_RAD = -0.349f; // -20 deg
        static constexpr float PITCH_MAX_ANGLE_RAD = 0.349f;  // +20 deg

        // Minimum and Maximum Airspeed Limits (in m/s)
        static constexpr float MIN_AIRSPEED_MPS = 20.0f; // 20 m/s = ~40 knots
        static constexpr float MAX_AIRSPEED_MPS = 62.0f; // 60 m/s = ~120 knots
        static constexpr float MAX_AIRSPEED_SLEW_RATE_MSS = 2.0f; // Max target speed change (m/s^2)

        // Maximum Altitude Change Rate (in m/s)
        static constexpr float MAX_ALTITUDE_DELTA_MPS = 2.50f; // 2.50m/s = 500 feet/min

        // PID output scale and shift to convert from [-1,1] normalized range to [0,100] motor range
        static constexpr float FBWB_PID_OUTPUT_SCALE = 50.0f;
        static constexpr float FBWB_PID_OUTPUT_SHIFT = 50.0f;

        // Assumed normalized range of RC Input to be [0, 100]
        static constexpr float MAX_RC_INPUT_VAL = 100.0f;
};
