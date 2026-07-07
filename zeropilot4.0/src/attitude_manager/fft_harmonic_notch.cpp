#include "fft_harmonic_notch.hpp"
#include <cmath>

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

FFTHarmonicNotch::FFTHarmonicNotch(ISystemUtils *systemUtilsDriver) : 
    systemUtilsDriver(systemUtilsDriver) {}

bool FFTHarmonicNotch::init(const FFTHarmonicNotchConfig& config) {
    // Validate configuration parameters
    if (config.sample_freq_hz <= 0.0f || config.bandwidth_hz <= 0.0f) {
        return false;
    }

    // Store configuration
    _config = config;

    // Calculate generic Attenuation (A) and Quality (Q) factors
    _A = powf(10.0f, -_config.attenuation_dB / 40.0f);
    _Q = _config.min_freq_hz / _config.bandwidth_hz;

    // Initialize CMSIS-DSP FFT Instance
    fftHandler->init(FFT_SIZE); // FILLMEIN: Replace arm_rfft_fast_init_f32 with FFT driver init
    
    _fftIndex = 0;

    // Pre-compute the Hanning Window to save FPU cycles during runtime
    for (int i = 0; i < FFT_SIZE; i++) {
        _hanningWindow[i] = 0.5f * (1.0f - systemUtilsDriver->cmsis_dsp_cosf(2.0f * M_PI * i / (FFT_SIZE - 1))); // FILLMEIN: Replace with arm_cos_f32 from SystemUtils
    }

    // Reset filter states and mark as initialized
    reset();
    _initialised = true;

    return true;
}

bool FFTHarmonicNotch::pushSample(float raw_gyro_sample) {
    if (!_initialised) return false;
    
    // Load sample into buffer
    _fftBuffer[_fftIndex++] = raw_gyro_sample;
    
    // If buffer is full, execute FFT
    if (_fftIndex >= FFT_SIZE) {
        float fftOutput[FFT_SIZE];
        float magnitudes[FFT_SIZE / 2]; // Real-valued signal has symmetric FFT output
        
        // 1. Apply Hanning window
        // FILLMEIN (for each sample in the buffer, buffer[i] *= _hanningWindow[i])
        for (int i = 0; i < FFT_SIZE; i++) {
            _fftBuffer[i] *= _hanningWindow[i];
        }
        
        // 2. Run FFT via CMSIS-DSP
        // FILLMEIN (call arm_rfft_fast_f32 with _fftHandler, _fftBuffer, fftOutput, 0)
        fftHandler->runFFT(_fftBuffer, fftOutput, 0); // 0 for time to freq domain
        
        // 3. Calculate Magnitudes via CMSIS-DSP [arm_cmplx_mag_f32]
        // FILLMEIN (call arm_cmplx_mag_f32 with fftOutput, magnitudes, FFT_SIZE / 2)
        fftHandler->computeMag(fftOutput, magnitudes, FFT_SIZE / 2);
        
        // 4. Find Peak Frequency Bin
        // Start searching at the bin corresponding to min_freq_hz to avoid physical flight dynamics
        // FILLMEIN (find idx for starting bin, and if 0 then set to 1 to avoid DC component)
        //          (then, loop through magnitudes to find the index w/ the maximum energy)
        uint8_t startBin = (uint8_t)(_config.min_freq_hz / (_config.sample_freq_hz / FFT_SIZE)); // Each bin covers _config.sample_freq_hz / FFT_SIZE hz
        if (startBin == 0) startBin = 1;
        uint8_t peakBin = startBin;
        float peak = magnitudes[peakBin];
        for (int i = startBin + 1; i < FFT_SIZE / 2; i++) {
            if (magnitudes[i] > peak) {
                peak = magnitudes[i];
                peakBin = i;
            }
        }
        
        // 5. Convert to Hz and update coefficients via updateFilters(peak_freq_hz)
        float peakFreq = (float)peakBin * (_config.sample_freq_hz / FFT_SIZE);
        updateFilters(peakFreq);
        
        // 6. Reset buffer index for next cycle
        _fftIndex = 0; // Reset buffer
        return true;   // FFT ran (buffer full)
    }
    
    return false; // Buffer not full yet
}

void FFTHarmonicNotch::updateFilters(float peak_freq_hz) {
    const float nyquist_limit = _config.sample_freq_hz * 0.48f;

    for (uint8_t i = 0; i < FFT_NOTCH_MAX_HARMONICS; i++) {
        // Check if this harmonic bit is enabled in the mask
        if (!((1U << i) & _config.harmonics_mask)) {
            _filters[i].enabled = false;
            continue;
        }

        float harmonic_freq = peak_freq_hz * (i + 1);

        // Disable filter if it exceeds Nyquist or drops below the minimum configured frequency
        if (harmonic_freq >= nyquist_limit || harmonic_freq < _config.min_freq_hz) {
            _filters[i].enabled = false;
            continue;
        }

        // Update coefficients for this specific harmonic
        _filters[i].updateCoefficients(systemUtilsDriver, _config.sample_freq_hz, harmonic_freq, _A, _Q);
        _filters[i].enabled = true;
    }
}

void FFTHarmonicNotch::apply(float& gx, float& gy, float& gz) {
    if (!_initialised) return;

    for (uint8_t i = 0; i < FFT_NOTCH_MAX_HARMONICS; i++) {
        if (_filters[i].enabled) {
            _filters[i].applyTriAxis(gx, gy, gz);
        }
    }
}

void FFTHarmonicNotch::reset() {
    for (uint8_t i = 0; i < FFT_NOTCH_MAX_HARMONICS; i++) {
        _filters[i].resetStates();
    }
}

// ---------------------------------------------------------
// Bi-Quadratic Filter Mathematical Implementation
// ---------------------------------------------------------

void FFTHarmonicNotch::BiquadState::updateCoefficients(ISystemUtils *systemUtilsDriver, float sample_freq, float center_freq, float A, float Q) {
    float omega = 2.0f * M_PI * center_freq / sample_freq;
    float sn = systemUtilsDriver->cmsis_dsp_sinf(omega); // FILLMEIN: Replace with arm_sin_f32(omega) from SystemUtils
    float cs = systemUtilsDriver->cmsis_dsp_cosf(omega); // FILLMEIN: Replace with arm_cos_f32(omega) from SystemUtils
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
