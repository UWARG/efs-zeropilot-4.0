#pragma once

#include <cstdint>

class IFFT {
    protected:
        IFFT() = default;
    public: 
        virtual bool init(uint16_t fftLen) = 0;

        virtual void runFFT(float *input_buffer, float *output_buffer, uint8_t direction) = 0;

        virtual void complexMag(const float *input_buffer, float *output_buffer, uint32_t numSamples) = 0; 
};
