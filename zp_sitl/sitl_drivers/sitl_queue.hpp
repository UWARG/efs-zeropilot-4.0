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
    
    ZP_ERROR_e get(T *message) override {
        if (q.empty()) return ZP_ERROR_FAIL;
        *message = q.front();
        q.pop();
        return ZP_ERROR_OK;
    }
    
    ZP_ERROR_e push(T *message) override {
        if (q.size() >= maxSize) return ZP_ERROR_MEMORY_OVERFLOW;
        q.push(*message);
        return ZP_ERROR_OK;
    }
    
    ZP_ERROR_e count(int &count_value) override {
        count_value = (int)q.size();
        return ZP_ERROR_OK;
    }
    
    ZP_ERROR_e remainingCapacity(int &capacity) override {
        capacity = (int)(maxSize - q.size());
        return ZP_ERROR_OK;
    }
};
