#pragma once
#include "queue_iface.hpp"
#include <queue>
#include <cstring>

class SITL_LogQueue : public IMessageQueue<char[100]> {
private:
    std::queue<std::string> q;
    size_t maxSize;
    
public:
    SITL_LogQueue(size_t max = 100) : maxSize(max) {}
    
    ZP_ERROR_e get(char (*message)[100]) override {
        if (q.empty()) return ZP_ERROR_FAIL;
        strncpy(*message, q.front().c_str(), 99);
        (*message)[99] = '\0';
        q.pop();
        return ZP_ERROR_OK;
    }
    
    ZP_ERROR_e push(char (*message)[100]) override {
        if (q.size() >= maxSize) return ZP_ERROR_MEMORY_OVERFLOW;
        q.push(std::string(*message));
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
