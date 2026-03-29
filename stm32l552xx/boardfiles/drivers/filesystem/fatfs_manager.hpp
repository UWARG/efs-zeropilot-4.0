#pragma once

#include "filesystem_iface.hpp"
#include "sd_fatfs_msgs.hpp"
#include "queue.hpp"

#define SD_SCHEDULING_RATE_HZ 10
#define SD_UPDATE_LOOP_DELAY_MS (1000 / SD_SCHEDULING_RATE_HZ)

class FatFSManager {
private:
    MessageQueue<FatFSReqMsg> *requestQueue;
    MessageQueue<FatFSReqBuff> *bufferQueue;

    MessageQueue<PollResult> **responseQueues; // Array of response queues for each manager ID
    void clearBufferQueue(ManId id, ReqType type);
public:
    explicit FatFSManager(MessageQueue<FatFSReqMsg> *reqQueue, MessageQueue<FatFSReqBuff> *buffQueue, MessageQueue<PollResult> *respQueues[static_cast<size_t>(ManId::COUNT)]);
    void run();
};
