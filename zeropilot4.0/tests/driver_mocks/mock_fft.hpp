#pragma once

#include "fft_iface.hpp"
#include <gmock/gmock.h>

class MockFFT : public IFFT {
    public:
        MOCK_METHOD(bool, init, (uint16_t fftLen), (override));
        MOCK_METHOD(void, runFFT, (float* input_buffer, float* output_buffer, uint8_t direction), (override));
        MOCK_METHOD(void, complexMag, (const float* input_buffer, float* output_buffer, uint32_t numSamples), (override));
};
