#pragma once

#include <cstddef>
#include "systemutils_iface.hpp"
#include "sd_msgs.hpp"
#include "queue_iface.hpp"

class SDManager {
public:
    explicit SDManager(
        ISystemUtils *systemUtilsDriver,
        IMessageQueue<SdReqMsg> *reqQueue,
        IMessageQueue<SdReqBuf> *bufQueue,
        IMessageQueue<PollResult> *respQueues[static_cast<size_t>(ManagerId_e::NUM_MANAGERS)]
    );
    
    // Services reqMsg, then drains any requests already waiting behind it
    void sdUpdate(SdReqMsg reqMsg);
    
private:
        ISystemUtils *systemUtilsDriver;
        IMessageQueue<SdReqMsg> *requestQueue;
        IMessageQueue<SdReqBuf> *bufferQueue;
        IMessageQueue<PollResult> **responseQueues; // Array of response queues for each manager ID
    
        uint8_t profilerId;
};
