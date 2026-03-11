#include "pid.hpp"
#include <error.h>;

// Constructor
PID::PID(float kp, float ki, float kd,
         float tau, float outputMinLim, float outputMaxLim,
         float integralMinLim, float integralMaxLim, float t) noexcept : 
            kp(kp), ki(ki), kd(kd), tau(tau), t(t),
            outputMinLim(outputMinLim), outputMaxLim(outputMaxLim),
            integralMinLim(integralMinLim), integralMaxLim(integralMaxLim)
{}

// Initialization method - Can be used as resetter
ZP_ERROR_e PID::pidInitState() noexcept {
    pidIntegral = 0.0f;
    prevError = 0.0f;
    pidDerivative = 0.0f;
    prevMeasurement = 0.0f;
    return ZP_ERROR_OK;
}

ZP_ERROR_e PID::setConstants(float newKp, float newKi, float newKd, float newTau) noexcept {
    kp = newKp;
    ki = newKi;
    kd = newKd;
    tau = newTau;
    return ZP_ERROR_OK;
}

// Update method
ZP_ERROR_e PID::pidOutput(float setpoint, float measurement, float *output) noexcept {
    if (output == nullptr) return ZP_ERROR_NULLPTR;
    
    // Calculate error
    float error = setpoint - measurement;

    // PID Proportional
    float pidProportional = kp * error;

    // PID Integral 
    pidIntegral += (0.5f * ki * t) * (error + prevError);

    // Anti-integral windup
    if (pidIntegral > integralMaxLim) { pidIntegral = integralMaxLim; }
    if (pidIntegral < integralMinLim) { pidIntegral = integralMinLim; }

    // PID Derivative with low-pass filter
    pidDerivative = ((-1 * 2.0f * kd * (measurement - prevMeasurement)) + ((2 * tau - t) * pidDerivative)) / ((2.0f * tau) + t);
    
    // PID control effort
    float pidControlEffort = pidProportional + pidIntegral + pidDerivative;
    
    // Clamp control effort output
    if (pidControlEffort > outputMaxLim) { pidControlEffort = outputMaxLim; }
    if (pidControlEffort < outputMinLim) { pidControlEffort = outputMinLim; }

    // Update previous values
    prevError = error;
    prevMeasurement = measurement;

    *output = pidControlEffort;
    return ZP_ERROR_OK;
}
