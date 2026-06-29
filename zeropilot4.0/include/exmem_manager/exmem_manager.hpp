#pragma once

#include <cstddef>
#include "filesystem_backend_iface.hpp"
#include "systemutils_iface.hpp"
#include "exmem_msgs.hpp"
#include "queue_iface.hpp"

#define SD_SCHEDULING_RATE_HZ 10
#define SD_UPDATE_LOOP_DELAY_MS (1000 / SD_SCHEDULING_RATE_HZ)

class ExMemManager {
    public:
        explicit ExMemManager(
            ISystemUtils *systemUtilsDriver,
            IFileSystemBackend *backend,
            IMessageQueue<ExMemReqMsg> *reqQueue,
            IMessageQueue<ExMemReqBuff> *buffQueue,
            IMessageQueue<PollResult> *respQueues[static_cast<size_t>(ManId::COUNT)]
        );

        void emUpdate(ExMemReqMsg reqMsg);

    private:
        ISystemUtils *systemUtilsDriver;
        IFileSystemBackend *backend;
        IMessageQueue<ExMemReqMsg> *requestQueue;
        IMessageQueue<ExMemReqBuff> *bufferQueue;

        IMessageQueue<PollResult> **responseQueues; // Array of response queues for each manager ID
        void clearBufferQueue(ManId id, ReqType type);

        uint8_t profilerId;
};
