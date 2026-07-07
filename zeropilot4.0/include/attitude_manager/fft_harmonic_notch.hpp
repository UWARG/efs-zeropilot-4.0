#pragma once

#include <cstdint>
#include "arm_math.h" // CMSIS-DSP library required for FFT

#define FFT_NOTCH_MAX_HARMONICS 8 
#define FFT_SIZE 256 // 256 points under 4kHz ODR gives ~15.625Hz frequency resolution

struct FFTHarmonicNotchConfig {
    float sample_freq_hz;    // IMU Sample Rate (e.g., 4000.0f)
    float min_freq_hz;       // Minimum frequency to track (e.g., 90.0f)
    float bandwidth_hz;      // Base notch width
    float attenuation_dB;    // Attenuation depth
    uint8_t harmonics_mask;  // Bitmask for harmonics
};

class FFTHarmonicNotch {
public:
    FFTHarmonicNotch() = default;

    // Initialize filters, compute Hanning window, setup FFT handler
    void init(const FFTHarmonicNotchConfig& config);

    // Push a raw sample into the FFT buffer. 
    // Returns true if the buffer filled and an FFT was calculated this cycle.
    bool pushSample(float raw_gyro_sample);

    // Apply the filter cascade to all three gyro axes in-place
    void apply(float& gx, float& gy, float& gz);

    // Reset filter delay states
    void reset();

private:
    // Calculates coefficients based on the new peak frequency
    void updateFilters(float peak_freq_hz);

    struct BiquadState {
        float b0, b1, b2, a1, a2;
        float x1_x = 0, x2_x = 0, y1_x = 0, y2_x = 0;
        float x1_y = 0, x2_y = 0, y1_y = 0, y2_y = 0;
        float x1_z = 0, x2_z = 0, y1_z = 0, y2_z = 0;
        bool enabled = false;

        void updateCoefficients(float sample_freq, float center_freq, float A, float Q);
        void applyTriAxis(float& gx, float& gy, float& gz);
        void resetStates();
    };

    FFTHarmonicNotchConfig _config;
    BiquadState _filters[FFT_NOTCH_MAX_HARMONICS];
    
    // DSP State
    arm_rfft_fast_instance_f32 _fftHandler;
    float _fftBuffer[FFT_SIZE];
    float _hanningWindow[FFT_SIZE];
    uint16_t _fftIndex = 0;
    
    float _A; 
    float _Q; 
    bool _initialised = false;
};
