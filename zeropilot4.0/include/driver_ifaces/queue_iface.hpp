#pragma once

#include "zp_error.h"

template <typename T>
class IMessageQueue {
    protected: 
        IMessageQueue() = default;
        
    public: 
        virtual ~IMessageQueue() = default;

        virtual int get(T *message) = 0;
        virtual int push(T *message) = 0;
        virtual int count(int &count_value) = 0;
        virtual int remainingCapacity(int &capacity) = 0;
};
