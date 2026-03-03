#pragma once

#include "flightmode.hpp"
#include "pid.hpp"

class FBWAMapping : public Flightmode {
    public:
        FBWAMapping(float control_iter_period_s) noexcept;

        RCMotorControlMessage_t runControl(RCMotorControlMessage_t controlInput, const DroneState_t &droneState) override;

        // Setter *roll* for PID consts
        void setRollPIDConstants(float newKp, float newKi, float newKd, float newTau) noexcept;

        // Setter for *pitch* PID consts
        void setPitchPIDConstants(float newKp, float newKi, float newKd, float newTau) noexcept;

        // Setter for *yaw* rudder mixing const
        void setYawRudderMixingConstant(float newMixingConst) noexcept;

        // Resetter for both roll and pitch PIDs
        void resetControlLoopState() noexcept;

        // Destructor
        ~FBWAMapping() noexcept override = default;

    private:
        // Roll and Pitch PID class objects
        PID rollPID;
        PID pitchPID;

        // Yaw rudder mixing constant
        float yawRudderMixingConst;

        // Roll integral limits
        static constexpr float ROLL_INTEGRAL_MIN_LIM = -50.0f;
        static constexpr float ROLL_INTEGRAL_MAX_LIM = +50.0f;
        
        // Pitch integral limits
        static constexpr float PITCH_INTEGRAL_MIN_LIM = -50.0f;
        static constexpr float PITCH_INTEGRAL_MAX_LIM = +50.0f;

        // Output limits (for control effort)
        static constexpr float OUTPUT_MIN = -50.0f;
        static constexpr float OUTPUT_MAX = +50.0f;

        // Roll and Pitch Angle Ranges (in radians)
        static constexpr float ROLL_MIN_ANGLE_RAD = -0.785f;  // -45 degrees
        static constexpr float ROLL_MAX_ANGLE_RAD = 0.785f;   // +45 degrees
        static constexpr float PITCH_MIN_ANGLE_RAD = -0.349f; // -20 degrees
        static constexpr float PITCH_MAX_ANGLE_RAD = 0.349f;  // +20 degrees

        // Assumed normalized range of RC Input to be [0, 100]
        static constexpr float MAX_RC_INPUT_VAL = 100.0f;
};
