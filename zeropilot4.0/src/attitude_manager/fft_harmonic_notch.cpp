#include "fft_harmonic_notch.hpp"
#include <cmath>

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

FFTHarmonicNotch::FFTHarmonicNotch(ISystemUtils *systemUtilsDriver, IFFT *fftDriver) : 
    systemUtilsDriver(systemUtilsDriver),
    fftDriver(fftDriver) {}

bool FFTHarmonicNotch::init(const FFTHarmonicNotchConfig &notchConfig) {
    // Validate configuration parameters
    if (notchConfig.sampleFreqHz <= 0.0f || notchConfig.bandwidthHz <= 0.0f) {
        return false;
    }

    // Store configuration
    config = notchConfig;

    // Calculate generic Attenuation (A) and Quality (Q) factors
    a = powf(10.0f, -config.attenuationDB / 40.0f);
    q = config.minFreqHz / config.bandwidthHz;

    // Initialize CMSIS-DSP FFT Instance
    if (fftDriver == nullptr || !fftDriver->init(FFT_WINDOW_SIZE)) {
        initialized = false;
        return false;
    }

    fftIndex = 0;

    // Pre-compute the Hanning Window to save FPU cycles during runtime
    for (int i = 0; i < FFT_WINDOW_SIZE; i++) {
        hanningWindow[i] = 0.5f * (1.0f - systemUtilsDriver->dspCosf(2.0f * M_PI * i / (FFT_WINDOW_SIZE - 1)));
    }

    // Reset filter states and mark as initialized
    reset();
    initialized = true;

    return true;
}

bool FFTHarmonicNotch::pushSample(float gx, float gy, float gz) {
    if (!initialized) return false;
    if (fftIndex >= FFT_WINDOW_SIZE) return false;

    // Accumulate RMS energy for this FFT window
    rmsX += gx * gx;
    rmsY += gy * gy;
    rmsZ += gz * gz;
    rmsCount++;

    // Use the dominant axis selected from the previous window
    float rawGyroSample;
    switch (dominantAxis) {
        case GyroAxis_e::X:
            rawGyroSample = gx;
            break;

        case GyroAxis_e::Y:
            rawGyroSample = gy;
            break;

        case GyroAxis_e::Z:
            rawGyroSample = gz;
            break;
    }

    // Load sample into buffer
    fftBuffer[fftIndex++] = rawGyroSample;

    // If buffer is full, execute FFT
    if (fftIndex >= FFT_WINDOW_SIZE) {

        // Pick strongest vibration axis for next FFT window
        if (rmsX >= rmsY && rmsX >= rmsZ) {
            dominantAxis = GyroAxis_e::X;
        }
        else if (rmsY >= rmsZ) {
            dominantAxis = GyroAxis_e::Y;
        }
        else {
            dominantAxis = GyroAxis_e::Z;
        }

        rmsX = 0.0f;
        rmsY = 0.0f;
        rmsZ = 0.0f;
        rmsCount = 0;

        float fftOutput[FFT_WINDOW_SIZE];
        float magnitudes[FFT_WINDOW_SIZE / 2]; // Real-valued signal has symmetric FFT output

        // 1. Apply Hanning window
        for (int i = 0; i < FFT_WINDOW_SIZE; i++) {
            fftBuffer[i] *= hanningWindow[i];
        }

        // 2. Run FFT via CMSIS-DSP
        fftDriver->runFFT(fftBuffer, fftOutput, 0); // 0 for time to freq domain

        // 3. Calculate Magnitudes
        fftDriver->complexMag(fftOutput, magnitudes, FFT_WINDOW_SIZE / 2);

        // 4. Find Peak Frequency Bin
        // Start searching at the bin corresponding to minFreqHz to avoid physical flight dynamics
        uint8_t startBin = (uint8_t)(config.minFreqHz / (config.sampleFreqHz / FFT_WINDOW_SIZE)); // Each bin covers config.sampleFreqHz / FFT_WINDOW_SIZE hz
        if (startBin == 0)
            startBin = 1;
        uint8_t peakBin = startBin;
        float peak = magnitudes[peakBin];
        for (int i = startBin + 1; i < FFT_WINDOW_SIZE / 2; i++) {
            if (magnitudes[i] > peak) {
                peak = magnitudes[i];
                peakBin = i;
            }
        }

        // 5. Convert to Hz and update coefficients
        float peakFreq = (float)peakBin * (config.sampleFreqHz / FFT_WINDOW_SIZE);
        updateFilters(peakFreq);

        // 6. Reset buffer index for next cycle
        fftIndex = 0; // Reset buffer
        return true;  // FFT ran (buffer full)
    }

    return false; // Buffer not full yet
}

void FFTHarmonicNotch::updateFilters(float peakFreqHz) {
    static constexpr float NYQUIST_SAFETY_FACTOR = 0.48f;
    const float NYQUIST_LIMIT = config.sampleFreqHz * NYQUIST_SAFETY_FACTOR;

    for (uint8_t i = 0; i < FFT_NOTCH_MAX_HARMONICS; i++) {
        // Check if this harmonic bit is enabled in the mask
        if (!((1U << i) & config.harmonicsMask)) {
            filters[i].enabled = false;
            continue;
        }

        float harmonicFreq = peakFreqHz * (i + 1);

        // Disable filter if it exceeds Nyquist or drops below the minimum configured frequency
        if (harmonicFreq >= NYQUIST_LIMIT || harmonicFreq < config.minFreqHz) {
            filters[i].enabled = false;
            continue;
        }

        // Update coefficients for this specific harmonic
        filters[i].updateCoefficients(systemUtilsDriver, config.sampleFreqHz, harmonicFreq, a, q);
        filters[i].enabled = true;
    }
}

void FFTHarmonicNotch::apply(float& gx, float& gy, float& gz) {
    if (!initialized) return;

    for (uint8_t i = 0; i < FFT_NOTCH_MAX_HARMONICS; i++) {
        if (filters[i].enabled) {
            filters[i].applyTriAxis(gx, gy, gz);
        }
    }
}

void FFTHarmonicNotch::reset() {
    for (uint8_t i = 0; i < FFT_NOTCH_MAX_HARMONICS; i++) {
        filters[i].resetStates();
    }

    rmsX = 0.0f;
    rmsY = 0.0f;
    rmsZ = 0.0f;
    rmsCount = 0;
    dominantAxis = GyroAxis_e::X;
}

// ---------------------------------------------------------
// Bi-Quadratic Filter Mathematical Implementation
// ---------------------------------------------------------

void FFTHarmonicNotch::BiquadState::updateCoefficients(ISystemUtils *systemUtilsDriver, float sample_freq, float center_freq, float A, float q) {
    float omega = 2.0f * M_PI * center_freq / sample_freq;
    float sn = systemUtilsDriver->dspSinf(omega);
    float cs = systemUtilsDriver->dspCosf(omega);
    float alpha = sn / (2.0f * q);

    float a0 = 1.0f + alpha * A;
    b0 = (1.0f + alpha) / a0;
    b1 = (-2.0f * cs) / a0;
    b2 = (1.0f - alpha) / a0;
    a1 = b1;
    a2 = (1.0f - alpha * A) / a0;
}

void FFTHarmonicNotch::BiquadState::applyTriAxis(float &gx, float &gy, float &gz) {
    // X Axis
    float outX = b0 * gx + b1 * x1X + b2 * x2X - a1 * y1X - a2 * y2X;
    x2X = x1X;
    x1X = gx;
    y2X = y1X;
    y1X = outX;
    gx = outX;

    // Y Axis
    float outY = b0 * gy + b1 * x1Y + b2 * x2Y - a1 * y1Y - a2 * y2Y;
    x2Y = x1Y;
    x1Y = gy;
    y2Y = y1Y;
    y1Y = outY;
    gy = outY;

    // Z Axis
    float outZ = b0 * gz + b1 * x1Z + b2 * x2Z - a1 * y1Z - a2 * y2Z;
    x2Z = x1Z;
    x1Z = gz;
    y2Z = y1Z;
    y1Z = outZ;
    gz = outZ;
}

void FFTHarmonicNotch::BiquadState::resetStates() {
    x1X = x2X = y1X = y2X = 0.0f;
    x1Y = x2Y = y1Y = y2Y = 0.0f;
    x1Z = x2Z = y1Z = y2Z = 0.0f;
}
