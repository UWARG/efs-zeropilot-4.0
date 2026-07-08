#include "fft_harmonic_notch.hpp"
#include <cmath>

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

FFTHarmonicNotch::FFTHarmonicNotch(ISystemUtils *systemUtilsDriver, IFFT *fftDriver) : 
    systemUtilsDriver(systemUtilsDriver),
    fftDriver(fftDriver) {}

bool FFTHarmonicNotch::init(const FFTHarmonicNotchConfig& config) {
    // Validate configuration parameters
    if (notchConfig.sample_freq_hz <= 0.0f || notchConfig.bandwidth_hz <= 0.0f) {
        return false;
    }

    // Store configuration
    config = notchConfig;

    // Calculate generic Attenuation (A) and Quality (Q) factors
    A = powf(10.0f, -config.attenuation_dB / 40.0f);
    Q = config.min_freq_hz / config.bandwidth_hz;

    // Initialize CMSIS-DSP FFT Instance
    if (fftDriver == nullptr || !fftDriver->init(FFT_WINDOW_SIZE)) {
        initialised = false;
        return false;
    }

    fftIndex = 0;

    // Pre-compute the Hanning Window to save FPU cycles during runtime
    for (int i = 0; i < FFT_WINDOW_SIZE; i++) {
        hanningWindow[i] = 0.5f * (1.0f - systemUtilsDriver->cmsisDspCosf(2.0f * M_PI * i / (FFT_WINDOW_SIZE - 1)));
    }

    // Reset filter states and mark as initialized
    reset();
    initialised = true;

    return true;
}

bool FFTHarmonicNotch::pushSample(float gx, float gy, float gz) {
    if (!initialised) return false;
    if (fftIndex >= FFT_WINDOW_SIZE) return false;

    // Accumulate RMS energy for this FFT window
    rmsX += gx * gx;
    rmsY += gy * gy;
    rmsZ += gz * gz;
    rmsCount++;

    // Use the dominant axis selected from the previous window
    float rawGyroSample;
    switch (dominantAxis) {
        case GyroAxis::X:
            rawGyroSample = gx;
            break;

        case GyroAxis::Y:
            rawGyroSample = gy;
            break;

        case GyroAxis::Z:
            rawGyroSample = gz;
            break;
    }

    // Load sample into buffer
    fftBuffer[fftIndex++] = rawGyroSample;

    // If buffer is full, execute FFT
    if (fftIndex >= FFT_WINDOW_SIZE) {

        // Pick strongest vibration axis for next FFT window
        if (rmsX >= rmsY && rmsX >= rmsZ) {
            dominantAxis = GyroAxis::X;
        }
        else if (rmsY >= rmsZ) {
            dominantAxis = GyroAxis::Y;
        }
        else {
            dominantAxis = GyroAxis::Z;
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
        fftDriver->computeMag(fftOutput, magnitudes, FFT_WINDOW_SIZE / 2);

        // 4. Find Peak Frequency Bin
        // Start searching at the bin corresponding to min_freq_hz to avoid physical flight dynamics
        uint8_t startBin = (uint8_t)(config.min_freq_hz / (config.sample_freq_hz / FFT_WINDOW_SIZE)); // Each bin covers config.sample_freq_hz / FFT_WINDOW_SIZE hz
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
        float peakFreq = (float)peakBin * (config.sample_freq_hz / FFT_WINDOW_SIZE);
        updateFilters(peakFreq);

        // 6. Reset buffer index for next cycle
        fftIndex = 0; // Reset buffer
        return true;  // FFT ran (buffer full)
    }

    return false; // Buffer not full yet
}

void FFTHarmonicNotch::updateFilters(float peak_freq_hz) {
    const float nyquist_limit = config.sample_freq_hz * 0.48f;

    for (uint8_t i = 0; i < FFT_NOTCH_MAX_HARMONICS; i++) {
        // Check if this harmonic bit is enabled in the mask
        if (!((1U << i) & config.harmonics_mask)) {
            filters[i].enabled = false;
            continue;
        }

        float harmonic_freq = peak_freq_hz * (i + 1);

        // Disable filter if it exceeds Nyquist or drops below the minimum configured frequency
        if (harmonic_freq >= nyquist_limit || harmonic_freq < config.min_freq_hz) {
            filters[i].enabled = false;
            continue;
        }

        // Update coefficients for this specific harmonic
        filters[i].updateCoefficients(systemUtilsDriver, config.sample_freq_hz, harmonic_freq, A, Q);
        filters[i].enabled = true;
    }
}

void FFTHarmonicNotch::apply(float& gx, float& gy, float& gz) {
    if (!_initialised) return;

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
    dominantAxis = GyroAxis::X;
}

// ---------------------------------------------------------
// Bi-Quadratic Filter Mathematical Implementation
// ---------------------------------------------------------

void FFTHarmonicNotch::BiquadState::updateCoefficients(ISystemUtils *systemUtilsDriver, float sample_freq, float center_freq, float A, float Q) {
    float omega = 2.0f * M_PI * center_freq / sample_freq;
    float sn = systemUtilsDriver->cmsisDspSinf(omega);
    float cs = systemUtilsDriver->cmsisDspCosf(omega);
    float alpha = sn / (2.0f * Q);

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
