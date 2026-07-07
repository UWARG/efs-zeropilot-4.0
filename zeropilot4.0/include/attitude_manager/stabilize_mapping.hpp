#pragma once

#include <cstdint>
#include "flightmode.hpp"
#include "pid.hpp"
#include "acro_mapping.hpp"

class StabilizeMapping : public Flightmode{
    public: 
        StabilizeMapping(float control_iter_period_s, AcroMapping &acro) noexcept;

        void activateFlightMode() override;

        RCMotorControlMessage_t runControl(RCMotorControlMessage_t controlInput, const DroneState_t &droneState) override;

        // Setter *roll* for PID consts
        void setRollPIDConstants(float newKp, float newKi, float newKd, float newTau, uint8_t newIMaxPct) noexcept;

        // Setter for *pitch* PID consts
        void setPitchPIDConstants(float newKp, float newKi, float newKd, float newTau, uint8_t newIMaxPct) noexcept;

        // Setter for *rollLimitAngle* and *pitchLimitAngle* in rad
        void setRollPitchLimitAngle(float newRollPitchLimitAngle) noexcept;

        // Resetter for all roll, pitch and yaw PIDs (needed for unit testing)
        void resetControlLoopState() noexcept;

        // Getter for PID objects
        PID *getRollPID() noexcept;
        PID *getPitchPID() noexcept;

        // Destructor
        ~StabilizeMapping() noexcept override = default;


    private: 
        static constexpr uint8_t ANGLE_LOOP_TO_INNER_LOOP_RATIO = 10;
        
         // Roll, and pitch PID class objects
        PID rollPID;
        PID pitchPID;

        // Values for roll, and pitch limits
        float rollPitchLimitAngle;

        AcroMapping &acroCLAW;

        uint16_t decimationCounter;

        float stabilizeRollCmd;
        float stabilizePitchCmd;

        // Output limits (for control effort)
        static constexpr float OUTPUT_MIN = -1.0f;
        static constexpr float OUTPUT_MAX = +1.0f;

        // PID output scale and shift to convert from [-1,1] normalized range to [0,100] motor range
        static constexpr float ACRO_PID_OUTPUT_SCALE = 50.0f;
        static constexpr float ACRO_PID_OUTPUT_SHIFT = 50.0f;

        // Assumed normalized range of RC Input to be [0, 100]
        static constexpr float MAX_RC_INPUT_VAL = 100.0f;
};