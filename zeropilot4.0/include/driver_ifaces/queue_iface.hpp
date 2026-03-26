#pragma once

#include "error.h"

template <typename T>
class IMessageQueue {
    protected:
        IMessageQueue() = default;

    public:
        virtual ~IMessageQueue() = default;

        virtual ZP_ERROR_e get(T *message) = 0;
        virtual ZP_ERROR_e push(T *message) = 0;
        virtual ZP_ERROR_e count(int *count_value) = 0;
        virtual ZP_ERROR_e remainingCapacity(int *capacity) = 0;
};
