#include "pid.hpp"

// Constructor
PID::PID(float kp, float ki, float kd, float tau,
         float outputMinLim, float outputMaxLim, uint8_t integralMaxPct,
         float t) noexcept : 
            kp(kp), ki(ki), kd(kd), tau(tau), t(t),
            outputMinLim(outputMinLim), outputMaxLim(outputMaxLim),
            integralMinLim((integralMaxPct / 100.0f) * outputMinLim),
            integralMaxLim((integralMaxPct / 100.0f) * outputMaxLim) {}

// Initialization method - Can be used as resetter
ZP_ERROR_e PID::pidInitState() noexcept {
    pidIntegral = 0.0f;
    prevError = 0.0f;
    pidDerivative = 0.0f;
    prevMeasurement = 0.0f;
    return ZP_ERROR_OK;
}

ZP_ERROR_e PID::setConstants(float newKp, float newKi, float newKd, float newTau, uint8_t newIMaxPct) noexcept {
    kp = newKp;
    ki = newKi;
    kd = newKd;
    tau = newTau;
    setIntegralMinLimPct(newIMaxPct);
    setIntegralMaxLimPct(newIMaxPct);
    return ZP_ERROR_OK;
}

void PID::setKp(float newKp) noexcept { kp = newKp; }
void PID::setKi(float newKi) noexcept { ki = newKi; }
void PID::setKd(float newKd) noexcept { kd = newKd; }
void PID::setTau(float newTau) noexcept { tau = newTau; }
void PID::setIntegralMinLimPct(uint8_t pct) noexcept { integralMinLim = (pct / 100.0f) * outputMinLim; }
void PID::setIntegralMaxLimPct(uint8_t pct) noexcept { integralMaxLim = (pct / 100.0f) * outputMaxLim; }

ZP_ERROR_e PID::pidOutput(float setpoint, float measurement, float &output) noexcept {
    ZP_ERROR_e result = ZP_ERROR_OK;

    // Calculate error
    float error = setpoint - measurement;

    // PID Proportional
    float pidProportional = kp * error;

    // PID Integral 
    pidIntegral += (0.5f * ki * t) * (error + prevError);

    // Anti-integral windup
    if (pidIntegral > integralMaxLim) { pidIntegral = integralMaxLim; }
    else if (pidIntegral < integralMinLim) { pidIntegral = integralMinLim; }

    // PID Derivative with low-pass filter
    // Note: Applying a check here to ensure t > 0 could be a future stability improvement
    pidDerivative = ((-1.0f * 2.0f * kd * (measurement - prevMeasurement)) + ((2.0f * tau - t) * pidDerivative)) / ((2.0f * tau) + t);
    
    // PID control effort
    float pidControlEffort = pidProportional + pidIntegral + pidDerivative;
    
    // Clamp control effort output
    if (pidControlEffort > outputMaxLim) { pidControlEffort = outputMaxLim; }
    else if (pidControlEffort < outputMinLim) { pidControlEffort = outputMinLim; }

    // Update previous values
    prevError = error;
    prevMeasurement = measurement;

    // Direct assignment to reference
    output = pidControlEffort;

    return result;
}