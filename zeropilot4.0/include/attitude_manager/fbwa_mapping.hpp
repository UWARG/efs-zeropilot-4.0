#pragma once

#include <cstdint>
#include "flightmode.hpp"
#include "pid.hpp"

class FBWAMapping : public Flightmode {
    public:
        FBWAMapping(float control_iter_period_s) noexcept;

        void activateFlightMode() override;

        RCMotorControlMessage_t runControl(RCMotorControlMessage_t controlInput, const DroneState_t &droneState) override;

        // Setter *roll* for PID consts
        void setRollPIDConstants(float newKp, float newKi, float newKd, float newTau, uint8_t newIMaxPct) noexcept;

        // Setter for *pitch* PID consts
        void setPitchPIDConstants(float newKp, float newKi, float newKd, float newTau, uint8_t newIMaxPct) noexcept;

        // Setter for *yaw* rudder mixing const
        void setYawRudderMixingConstant(float newMixingConst) noexcept;

        // Setter for *rollLimitRad*
        void setRollLimitDeg(float newRollLimitDeg) noexcept;

        // Setter for *pitchLimitMaxRad*
        void setPitchLimitMaxDeg(float newPitchLimitMaxDeg) noexcept;

        // Setter for *pitchLimitMinRad*
        void setPitchLimitMinDeg(float newPitchLimitMinDeg) noexcept;

        // Resetter for both roll and pitch PIDs
        void resetControlLoopState() noexcept;

        // Getter for PID objects
        PID *getRollPID() noexcept;
        PID *getPitchPID() noexcept;

        // Destructor
        ~FBWAMapping() noexcept override = default;

    private:
        // Roll and Pitch PID class objects
        PID rollPID;
        PID pitchPID;

        // Yaw rudder mixing constant
        float yawRudderMixingConst;

        // Values for roll/pitch limits
        float rollLimitRad;
        float pitchLimitMaxRad;
        float pitchLimitMinRad;

        // Output limits (for control effort)
        static constexpr float OUTPUT_MIN = -1.0f;
        static constexpr float OUTPUT_MAX = +1.0f;

        // Integral limits (to prevent windup)
        static constexpr float INTEGRAL_MIN = -0.5f;
        static constexpr float INTEGRAL_MAX = +0.5f;

        // PID output scale and shift to convert from [-1,1] normalized range to [0,100] motor range
        static constexpr float FBWA_PID_OUTPUT_SCALE = 50.0f;
        static constexpr float FBWA_PID_OUTPUT_SHIFT = 50.0f;

        // Assumed normalized range of RC Input to be [0, 100]
        static constexpr float MAX_RC_INPUT_VAL = 100.0f;
};
