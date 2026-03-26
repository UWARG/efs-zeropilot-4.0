#pragma once

#include "flightmode.hpp"
#include "pid.hpp"

class FBWAMapping : public Flightmode {
    public:
        FBWAMapping(float control_iter_period_s) noexcept;


        void activateFlightMode() override;

        ZP_ERROR_e runControl(RCMotorControlMessage_t *controlOutput, const RCMotorControlMessage_t controlInput, const DroneState_t &droneState) override;
        
        // Setter *roll* for PID consts
        ZP_ERROR_e setRollPIDConstants(float newKp, float newKi, float newKd, float newTau) noexcept;

        // Setter for *pitch* PID consts
        ZP_ERROR_e setPitchPIDConstants(float newKp, float newKi, float newKd, float newTau) noexcept;

        // Setter for *yaw* rudder mixing const
        ZP_ERROR_e setYawRudderMixingConstant(float newMixingConst) noexcept;

        // Resetter for both roll and pitch PIDs
        ZP_ERROR_e resetControlLoopState() noexcept;

        // Destructor
        ~FBWAMapping() noexcept override = default;

    private:
        // Roll and Pitch PID class objects
        PID rollPID;
        PID pitchPID;

        // Yaw rudder mixing constant
        float yawRudderMixingConst;

        // Output limits (for control effort)
        static constexpr float OUTPUT_MIN = -1.0f;
        static constexpr float OUTPUT_MAX = +1.0f;

        // Integral limits (to prevent windup)
        static constexpr float INTEGRAL_MIN = -0.5f;
        static constexpr float INTEGRAL_MAX = +0.5f;

        // Roll and Pitch Angle Ranges (in radians)
        static constexpr float ROLL_MIN_ANGLE_RAD = -0.785f;  // -45 degrees
        static constexpr float ROLL_MAX_ANGLE_RAD = 0.785f;   // +45 degrees
        static constexpr float PITCH_MIN_ANGLE_RAD = -0.349f; // -20 degrees
        static constexpr float PITCH_MAX_ANGLE_RAD = 0.349f;  // +20 degrees

        // PID output scale and shift to convert from [-1,1] normalized range to [0,100] motor range
        static constexpr float FBWA_PID_OUTPUT_SCALE = 50.0f;
        static constexpr float FBWA_PID_OUTPUT_SHIFT = 50.0f;

        // Assumed normalized range of RC Input to be [0, 100]
        static constexpr float MAX_RC_INPUT_VAL = 100.0f;
};
