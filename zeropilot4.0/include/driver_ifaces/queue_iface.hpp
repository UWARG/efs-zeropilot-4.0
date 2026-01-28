#pragma once

template <typename T>
class IMessageQueue {
    protected: 
        IMessageQueue() = default;
        
    public: 
        virtual ~IMessageQueue() = default;
        
        virtual int get(T *message) = 0;
        virtual int push(T *message) = 0;
        virtual int count() = 0;
        virtual int remainingCapacity() = 0;
};
