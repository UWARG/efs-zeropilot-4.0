#pragma once

#include <cstdint>
#include "flightmode.hpp"
#include "pid.hpp"

class ACROMapping : public Flightmode{
    public: 
        ACROMapping(float control_iter_period_s) noexcept;

        void activateFlightMode() override;

        RCMotorControlMessage_t runControl(RCMotorControlMessage_t controlInput, const DroneState_t &droneState) override;

        // Setter *roll* for PID consts
        void setRollPIDConstants(float newKp, float newKi, float newKd, float newTau, uint8_t newIMaxPct) noexcept;

        // Setter for *pitch* PID consts
        void setPitchPIDConstants(float newKp, float newKi, float newKd, float newTau, uint8_t newIMaxPct) noexcept;

        // Setter for *yaw* PID consts
        void setYawPIDConstants(float newKp, float newKi, float newKd, float newTau, uint8_t newIMaxPct) noexcept;

        // Setter for *rollLimitRad*
        void setRollLimitDeg(float newRollLimitDeg) noexcept;

         // Setter for *pitchLimitRad*
        void setPitchLimitDeg(float newPitchLimitDeg) noexcept;

         // Setter for *yawLimitRad*
        void setYawLimitDeg(float newYawLimitDeg) noexcept;

        // Resetter for all roll, pitch and yaw PIDs (needed for unit testing)
        void resetControlLoopState() noexcept;

        // Getter for PID objects
        PID *getRollPID() noexcept;
        PID *getPitchPID() noexcept;
        PID *getYawPID() noexcept;

        // Destructor
        ~ACROMapping() noexcept override = default;


    private: 
         // Roll, pitch and yaw PID class objects
        PID rollPID;
        PID pitchPID;
        PID yawPID;

        // Values for roll, pitch and yaw limits
        float rollLimitRad;
        float pitchLimitRad;
        float yawLimitRad;

        // Output limits (for control effort)
        static constexpr float OUTPUT_MIN = -1.0f;
        static constexpr float OUTPUT_MAX = +1.0f;

        // PID output scale and shift to convert from [-1,1] normalized range to [0,100] motor range
        static constexpr float FBWA_PID_OUTPUT_SCALE = 50.0f;
        static constexpr float FBWA_PID_OUTPUT_SHIFT = 50.0f;

        // Assumed normalized range of RC Input to be [0, 100]
        static constexpr float MAX_RC_INPUT_VAL = 100.0f;
}