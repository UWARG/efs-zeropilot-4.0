#pragma once

#include <cstdint>
#include "flightmode.hpp"
#include "pid.hpp"
#include "acro_mapping.hpp"

class STABILIZEMapping : public Flightmode{
    public: 
        STABILIZEMapping(float control_iter_period_s_stabilize, float control_iter_period_s_acro) noexcept;

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
        
        // Setter for ACRO *roll* for PID consts
        void setAcroRollPIDConstants(float newKp, float newKi, float newKd, float newTau, uint8_t newIMaxPct) noexcept;
        
        // Setter for ACRO *pitch* PID consts
        void setAcroPitchPIDConstants(float newKp, float newKi, float newKd, float newTau, uint8_t newIMaxPct) noexcept;
        
        // Setter for ACRO *yaw* PID consts
        void setAcroYawPIDConstants(float newKp, float newKi, float newKd, float newTau, uint8_t newIMaxPct) noexcept;
        
        // Setter for ACRO *rollLimitRate* in rad / s
        void setAcroRollLimitRate(float newRollLimitRate) noexcept;
        
        // Setter for ACRO *pitchLimitRate* in rad / s
        void setAcroPitchLimitRate(float newPitchLimitRate) noexcept;
        
        // Setter for ACRO *yawLimitRate* in rad / s
        void setAcroYawLimitRate(float newYawLimitRate) noexcept;
        
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

        ACROMapping acroCLAW;

        // Output limits (for control effort)
        static constexpr float OUTPUT_MIN = -1.0f;
        static constexpr float OUTPUT_MAX = +1.0f;

        // PID output scale and shift to convert from [-1,1] normalized range to [0,100] motor range
        static constexpr float ACRO_PID_OUTPUT_SCALE = 50.0f;
        static constexpr float ACRO_PID_OUTPUT_SHIFT = 50.0f;

        // Assumed normalized range of RC Input to be [0, 100]
        static constexpr float MAX_RC_INPUT_VAL = 100.0f;
};