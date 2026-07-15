#pragma once

#include <cstdint>

class IMathUtils {
    protected:
        IMathUtils() = default;

    public:
        virtual ~IMathUtils() = default;

        virtual float dspSinf(float x) = 0;
        virtual float dspCosf(float x) = 0;
};
