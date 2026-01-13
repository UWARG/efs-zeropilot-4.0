#pragma once

#include "flightmode.hpp"
#include "attitude_manager.hpp"
#include "pid.hpp"

class FBWAMapping : public Flightmode {
    public:
        FBWAMapping() noexcept;

        RCMotorControlMessage_t runControl(RCMotorControlMessage_t controlInput, const DroneState_t &droneState) override;

        // Setter *roll* for PID consts
        void setRollPIDConstants(float newKp, float newKi, float newKd, float newTau) noexcept;

        // Setter for *pitch* PID consts
        void setPitchPIDConstants(float newKp, float newKi, float newKd, float newTau) noexcept;

        // Destructor
        ~FBWAMapping() noexcept override;
    
    private:

        // Roll integral limits
        inline const static float ROLL_INTEGRAL_MIN_LIM = -50.0f;
        inline const static float ROLL_INTEGRAL_MAX_LIM = +50.0f;
        
        // Pitch integral limits
        inline const static float PITCH_INTEGRAL_MIN_LIM = -50.0f;
        inline const static float PITCH_INTEGRAL_MAX_LIM = +50.0f;

        // Output limits (for control effort)
        inline const static float OUTPUT_MIN = -50.0f;
        inline const static float OUTPUT_MAX = +50.0f;

        // Roll and Pitch PID class objects
        PID rollPID;
        PID pitchPID;

        // Roll and Pitch Angle Ranges (in radians)
        inline const static float ROLL_MIN_ANGLE_RAD = -0.785; 	// -45 degrees
        inline const static float ROLL_MAX_ANGLE_RAD = 0.785; 	// +45 degrees
        inline const static float PITCH_MIN_ANGLE_RAD = -0.349; // -20 degrees
        inline const static float PITCH_MAX_ANGLE_RAD = 0.349;	// +20 degrees

        // Assumed normalized range of RC Input to be [0, 100]
        inline const static uint8_t MAX_RC_INPUT_VAL = 100;
};
