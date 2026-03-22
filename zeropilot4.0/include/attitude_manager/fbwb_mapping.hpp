#pragma once

#include "fbwa_mapping.hpp"
#include "pid.hpp"

class FBWBMapping : public FBWAMapping {
    public:
        FBWBMapping(float control_iter_period_s) noexcept;

        void activateFlightMode() override;

        RCMotorControlMessage_t runControl(RCMotorControlMessage_t controlInput, const DroneState_t &droneState) override;

        void setTotalEnergyPIDConstants(float newKp, float newKi, float newKd, float newTau, uint8_t newIMaxPct) noexcept;
        void setEnergyBalancePIDConstants(float newKp, float newKi, float newKd, float newTau, uint8_t newIMaxPct) noexcept;

        void resetControlLoopState() noexcept;

        ~FBWBMapping() noexcept override = default;

    private:
        float calculateSpecificKE(float airspeed_mps);

        // TECS outer PID control loops
        PID totalEnergyPID;
        PID energyBalancePID;

        // TECS State Variables
        bool isInitialized;
        float targetAltitude_m;
        float targetAirspeed_mps;
        float currentPitchSetpoint;
        float currentThrottleOutput_pct;
        uint32_t outerLoopSchedulingCounter;
        float dt_s;

        // Ratio between scheduling for inner loop and outer loop
        static constexpr uint8_t OUTER_LOOP_DIVIDER = 10;

        // Constants
        static constexpr float GRAVITY_MSS = 9.81f;

        // Minimum and Maximum Airspeed Limits (in m/s)
        static constexpr float MIN_AIRSPEED_MPS = 20.0f; // 20 m/s = ~40 knots
        static constexpr float MAX_AIRSPEED_MPS = 62.0f; // 60 m/s = ~120 knots
        static constexpr float MAX_AIRSPEED_SLEW_RATE_MSS = 1.0f; // Max target speed change (m/s^2)
        static constexpr float AIRSPEED_WEIGHT = 0.5f; // TECS energy balance weighting of KE relative to PE

        // Maximum Altitude Change Rate (in m/s)
        static constexpr float MAX_ALTITUDE_DELTA_MPS = 2.50f; // 2.50m/s = 500 feet/min
};
