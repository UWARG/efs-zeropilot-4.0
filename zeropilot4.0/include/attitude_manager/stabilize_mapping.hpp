#pragma once

#include <cstdint>
#include "flightmode.hpp"
#include "pid.hpp"
#include "acro_mapping.hpp"

class STABILIZEMapping : public Flightmode{
    public: 
        STABILIZEMapping(float stabilize_control_iter_period_s, float acro_control_iter_period_s, ACROMapping &acro) noexcept;

        void activateFlightMode() override;

        RCMotorControlMessage_t runControl(RCMotorControlMessage_t controlInput, const DroneState_t &droneState) override;

        // Setter *roll* for PID consts
        void setRollPIDConstants(float newKp, float newKi, float newKd, float newTau, uint8_t newIMaxPct) noexcept;

        // Setter for *pitch* PID consts
        void setPitchPIDConstants(float newKp, float newKi, float newKd, float newTau, uint8_t newIMaxPct) noexcept;

        // Setter for *rollLimitAngle* in rad
        void setRollLimitAngle(float newRollLimitAngle) noexcept;

        // Setter for *pitchLimitAngle* in rad
        void setPitchLimitAngle(float newPitchLimitAngle) noexcept;
        
        // Resetter for all roll, pitch and yaw PIDs (needed for unit testing)
        void resetControlLoopState() noexcept;

        // Getter for PID objects
        PID *getRollPID() noexcept;
        PID *getPitchPID() noexcept;

        // Destructor
        ~STABILIZEMapping() noexcept override = default;


    private: 
         // Roll, and pitch PID class objects
        PID rollPID;
        PID pitchPID;

        // Values for roll, and pitch limits
        float rollLimitAngle;
        float pitchLimitAngle;

        ACROMapping &acroCLAW;

        uint16_t decimationFactor;
        uint16_t decimationCounter;

        float stabilizeRollCmd;
        float stabilizePitchCmd;

        static constexpr uint16_t computeDecimation(float stabilize_control_iter_period_s, float acro_control_iter_period_s) noexcept
        {
            return (stabilize_control_iter_period_s != 0) ? (acro_control_iter_period_s / stabilize_control_iter_period_s) : 1;
        }

        // Output limits (for control effort)
        static constexpr float OUTPUT_MIN = -1.0f;
        static constexpr float OUTPUT_MAX = +1.0f;

        // PID output scale and shift to convert from [-1,1] normalized range to [0,100] motor range
        static constexpr float ACRO_PID_OUTPUT_SCALE = 50.0f;
        static constexpr float ACRO_PID_OUTPUT_SHIFT = 50.0f;

        // Assumed normalized range of RC Input to be [0, 100]
        static constexpr float MAX_RC_INPUT_VAL = 100.0f;
};