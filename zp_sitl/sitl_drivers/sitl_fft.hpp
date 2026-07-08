#pragma once

#include "arm_math.h"
#include "fft_iface.hpp"

class SITL_FFT : public IFFT {
    public:
        bool init(uint16_t fftLen) override {
            return arm_rfft_fast_init_f32(&fft, fftLen) == ARM_MATH_SUCCESS;
        }
        void runFFT(float *in, float *out, uint8_t dir) override {
            if (dir != 0 && dir != 1) return;
            arm_rfft_fast_f32(&fft, in, out, dir);
        }
        void computeMag(const float *in, float *out, uint32_t n) override {
            arm_cmplx_mag_f32(in, out, n);
        }
        
    private:
        arm_rfft_fast_instance_f32 fft;
};