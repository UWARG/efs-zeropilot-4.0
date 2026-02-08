#pragma once
#include "queue_iface.hpp"
#include <queue>

template <typename T>
class SITL_Queue : public IMessageQueue<T> {
private:
    std::queue<T> q;
    size_t maxSize;
    
public:
    SITL_Queue(size_t max = 100) : maxSize(max) {}
    
    int get(T *message) override {
        if (q.empty()) return -1;
        *message = q.front();
        q.pop();
        return 0;
    }
    
    int push(T *message) override {
        if (q.size() >= maxSize) return -1;
        q.push(*message);
        return 0;
    }
    
    int count() override {
        return (int)q.size();
    }
    
    int remainingCapacity() override {
        return (int)(maxSize - q.size());
    }
};
