#pragma once

#include <cstddef>
#include "filesystem_backend_iface.hpp"
#include "systemutils_iface.hpp"
#include "exmem_msgs.hpp"
#include "queue_iface.hpp"

static constexpr uint8_t EXMEM_SCHEDULING_RATE_HZ = 10;
static constexpr uint8_t SD_UPDATE_LOOP_DELAY_MS = (1000 / EXMEM_SCHEDULING_RATE_HZ);

class ExMemManager {
    public:
        explicit ExMemManager(
            ISystemUtils *systemUtilsDriver,
            IFileSystemBackend *fsBackend,
            IMessageQueue<ExMemReqMsg> *reqQueue,
            IMessageQueue<ExMemReqBuf> *bufQueue,
            IMessageQueue<PollResult> *respQueues[static_cast<size_t>(ManagerId_e::NUM_MANAGERS)]
        );

        void emUpdate(ExMemReqMsg reqMsg);

    private:
        ISystemUtils *systemUtilsDriver;
        IFileSystemBackend *fsBackend;
        IMessageQueue<ExMemReqMsg> *requestQueue;
        IMessageQueue<ExMemReqBuf> *bufferQueue;

        IMessageQueue<PollResult> **responseQueues; // Array of response queues for each manager ID
        void clearBufferQueue(ManagerId_e id, ReqType_e type);

        uint8_t profilerId;
};
