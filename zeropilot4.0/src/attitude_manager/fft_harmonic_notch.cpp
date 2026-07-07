#include "fft_harmonic_notch.hpp"
#include <cmath>

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

// ---------------------------------------------------------
// Bi-Quadratic Filter Mathematical Implementation
// ---------------------------------------------------------

void FFTHarmonicNotch::BiquadState::updateCoefficients(float sample_freq, float center_freq, float A, float Q) {
    float omega = 2.0f * M_PI * center_freq / sample_freq;
    float sn = sinf(omega); // FILLMEIN: Replace with arm_sin_f32(omega) from SystemUtils
    float cs = cosf(omega); // FILLMEIN: Replace with arm_cos_f32(omega) from SystemUtils
    float alpha = sn / (2.0f * Q);

    float a0 = 1.0f + alpha * A;
    b0 = (1.0f + alpha) / a0;
    b1 = (-2.0f * cs) / a0;
    b2 = (1.0f - alpha) / a0;
    a1 = b1; 
    a2 = (1.0f - alpha * A) / a0;
}

void FFTHarmonicNotch::BiquadState::applyTriAxis(float& gx, float& gy, float& gz) {
    // X Axis
    float outX = b0 * gx + b1 * x1_x + b2 * x2_x - a1 * y1_x - a2 * y2_x;
    x2_x = x1_x; x1_x = gx; y2_x = y1_x; y1_x = outX;
    gx = outX;

    // Y Axis
    float outY = b0 * gy + b1 * x1_y + b2 * x2_y - a1 * y1_y - a2 * y2_y;
    x2_y = x1_y; x1_y = gy; y2_y = y1_y; y1_y = outY;
    gy = outY;

    // Z Axis
    float outZ = b0 * gz + b1 * x1_z + b2 * x2_z - a1 * y1_z - a2 * y2_z;
    x2_z = x1_z; x1_z = gz; y2_z = y1_z; y1_z = outZ;
    gz = outZ;
}

void FFTHarmonicNotch::BiquadState::resetStates() {
    x1_x = x2_x = y1_x = y2_x = 0.0f;
    x1_y = x2_y = y1_y = y2_y = 0.0f;
    x1_z = x2_z = y1_z = y2_z = 0.0f;
}
