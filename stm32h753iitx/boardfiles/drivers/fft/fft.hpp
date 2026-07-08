#pragma once

#include "arm_math.h"
#include "fft_iface.hpp"

class FFT : public IFFT {
    public:
        bool init(uint16_t fftLen) override;

        void runFFT(float *input_buffer, float *output_buffer, uint8_t direction) override;

        void complexMag(const float *input_buffer, float *output_buffer, uint32_t numSamples) override; 

    private:
        arm_rfft_fast_instance_f32 fftInstance;
};
