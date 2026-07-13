#pragma once

#include <cstdint>
#include "systemutils_iface.hpp"
#include "fft_iface.hpp"

#define FFT_NOTCH_MAX_HARMONICS 16

enum class GyroAxis_e {
    X,
    Y,
    Z
};

struct FFTHarmonicNotchConfig {
    bool enabled;           // Enable/disable the FFT harmonic notch filter
    uint16_t fftWindowSize; // FFT window size (must be a power of 2)
    float sampleFreqHz;     // IMU Sample Rate
    float minFreqHz;        // Minimum frequency to track
    float bandwidthHz;      // Base notch width
    float attenuationDB;    // Attenuation depth
    uint8_t harmonicsMask;  // Bitmask for harmonics
};

class FFTHarmonicNotch {
    public:
        FFTHarmonicNotch(ISystemUtils *systemUtilsDriver, IFFT *fftDriver);
        
        // Initialize filters, compute Hanning window, setup FFT handler
        bool init(const FFTHarmonicNotchConfig& notchConfig);
        
        // Push a raw sample into the FFT buffer. 
        // Returns true if the buffer filled and an FFT was calculated this cycle.
        bool pushSample(float gx, float gy, float gz);
        
        // Apply the filter cascade to all three gyro axes in-place
        void apply(float& gx, float& gy, float& gz);
        
        // Reset filter delay states
        void reset();
    
    private:
        static constexpr uint16_t FFT_MAX_WINDOW_SIZE = 1024;

        ISystemUtils *systemUtilsDriver;

        // Calculates coefficients based on the new peak frequency
        void updateFilters(float peakFreqHz);

        struct BiquadState {
            float b0, b1, b2, a1, a2;
            float x1X = 0, x2X = 0, y1X = 0, y2X = 0;
            float x1Y = 0, x2Y = 0, y1Y = 0, y2Y = 0;
            float x1Z = 0, x2Z = 0, y1Z = 0, y2Z = 0;
            bool enabled = false;

            void updateCoefficients(ISystemUtils *systemUtilsDriver, float sample_freq, float center_freq, float A, float Q);
            void applyTriAxis(float &gx, float &gy, float &gz);
            void resetStates();
        };

        FFTHarmonicNotchConfig config;
        BiquadState filters[FFT_NOTCH_MAX_HARMONICS];
        
        // DSP State
        IFFT *fftDriver;
        float fftBuffer[FFT_MAX_WINDOW_SIZE];
        float hanningWindow[FFT_MAX_WINDOW_SIZE];
        float fftOutput[FFT_MAX_WINDOW_SIZE];
        float magnitudes[FFT_MAX_WINDOW_SIZE / 2]; // Real-valued signal has symmetric FFT output
        uint16_t fftIndex = 0;

        GyroAxis_e dominantAxis = GyroAxis_e::X;
        float rmsX = 0.0f;
        float rmsY = 0.0f;
        float rmsZ = 0.0f;
        uint16_t rmsCount = 0;
        
        float a; 
        float q; 
        bool initialized = false;
};
