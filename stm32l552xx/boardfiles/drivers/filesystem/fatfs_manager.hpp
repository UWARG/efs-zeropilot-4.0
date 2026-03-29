#pragma once

#include "filesystem_iface.hpp"
#include "sd_fatfs_msgs.hpp"
#include "queue.hpp"

class FatFSManager {
private:
    MessageQueue<FatFSReqMsg> *requestQueue;
    MessageQueue<FatFSReqBuff> *bufferQueue;

    MessageQueue<PollResult> **responseQueues; // Array of response queues for each manager ID
    void clearBufferQueue(ManId id, ReqType type);
public:
    explicit FatFSManager(MessageQueue<FatFSReqMsg> *reqQueue, MessageQueue<FatFSReqBuff> *buffQueue, MessageQueue<PollResult> *respQueues[ManId::COUNT]);
    void run();
};
