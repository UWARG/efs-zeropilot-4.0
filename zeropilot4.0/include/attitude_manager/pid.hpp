#pragma once

#include <cstdint>

class PID {
    public:
        // PID object constructor
        PID(float kp, float ki, float kd, float tau, 
            float outputMinLim, float outputMaxLim, uint8_t integralMaxPct,
            float t) noexcept;

        // PID object's state var initialized (or reset)
        ZP_ERROR_e pidInitState() noexcept;

        // For the PID roll & pitch consts -> may choose these to be optimized real-time dep. on optimization alg. chosen
        ZP_ERROR_e setConstants(float newKp, float newKi, float newKd, float newTau, uint8_t newIMaxPct) noexcept;

        // Setter for individual constants to be passed to ZP_PARAM layer
        ZP_ERROR_e setKp(float newKp) noexcept;
        ZP_ERROR_e setKi(float newKi) noexcept;
        ZP_ERROR_e setKd(float newKd) noexcept;
        ZP_ERROR_e setTau(float newTau) noexcept;
        ZP_ERROR_e setIntegralMinLimPct(uint8_t pct) noexcept;
        ZP_ERROR_e setIntegralMaxLimPct(uint8_t pct) noexcept;

        // Computes PID for a measurement with its desired setpoint passed in
        ZP_ERROR_e pidOutput(float setpoint, float measurement, float *output) noexcept;


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
