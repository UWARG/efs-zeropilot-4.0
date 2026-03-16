#pragma once

#include <cstdint>

class PID {
    public:
        // PID object constructor
        PID(float kp, float ki, float kd, float tau, 
            float outputMinLim, float outputMaxLim, uint8_t integralMaxPct,
            float t) noexcept;

        // PID object's state var initialized (or reset)
        void pidInitState() noexcept;

        // For the PID roll & pitch consts -> may choose these to be optimized real-time dep. on optimization alg. chosen
        void setConstants(float newKp, float newKi, float newKd, float newTau, uint8_t newIMaxPct) noexcept;

        // Setter for individual constants to be passed to ZP_PARAM layer
        void setKp(float newKp) noexcept;
        void setKi(float newKi) noexcept;
        void setKd(float newKd) noexcept;
        void setTau(float newTau) noexcept;
        void setIntegralMinLimPct(uint8_t pct) noexcept;
        void setIntegralMaxLimPct(uint8_t pct) noexcept;

        // Computes PID for a measurement with its desired setpoint passed in
        float pidOutput(float setpoint, float measurement) noexcept;


    private:
        // Gains
        float kp, ki, kd;      // PID constants
        float tau;             // Derivative low-pass filter constant
        float t;               // Sample time (set to AM_CONTROL_LOOP_PERIOD_S)

        // Output and Integral Limits
        float outputMinLim, outputMaxLim;       // Output limits
        float integralMinLim, integralMaxLim;   // integral limits

        // State variables
        float pidDerivative, pidIntegral;
        float prevError, prevMeasurement;
};
