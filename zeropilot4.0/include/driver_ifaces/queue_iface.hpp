#pragma once
#include "zp_error.h"

template <typename T>
class IMessageQueue {
    protected: 
        IMessageQueue() = default;
        
    public: 
        virtual ~IMessageQueue() = default;

        virtual ZP_Error get(T *message) = 0;
        virtual ZP_Error push(T *message) = 0;
        virtual ZP_Error count(int &count_value) = 0;
        virtual ZP_Error remainingCapacity(int &capacity) = 0;
};
