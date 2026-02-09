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
    
    int get(char (*message)[100]) override {
        if (q.empty()) return -1;
        strncpy(*message, q.front().c_str(), 99);
        (*message)[99] = '\0';
        q.pop();
        return 0;
    }
    
    int push(char (*message)[100]) override {
        if (q.size() >= maxSize) return -1;
        q.push(std::string(*message));
        return 0;
    }
    
    int count() override {
        return (int)q.size();
    }
    
    int remainingCapacity() override {
        return (int)(maxSize - q.size());
    }
};
