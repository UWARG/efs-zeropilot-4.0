#include "fft.hpp"

bool FFT::init(uint16_t fftLen) {
    return arm_rfft_fast_init_f32(&fftInstance, fftLen) == ARM_MATH_SUCCESS ? true : false;
}

void FFT::runFFT(float *input_buffer, float *output_buffer, uint8_t direction) {
    if (direction != 0 && direction != 1) return;
    arm_rfft_fast_f32(&fftInstance, input_buffer, output_buffer, direction);
}

void FFT::complexMag(const float *input_buffer, float *output_buffer, uint32_t numSamples) {
    arm_cmplx_mag_f32(input_buffer, output_buffer, numSamples);
}
