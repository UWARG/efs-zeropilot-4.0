#pragma once

#include "queue_iface.hpp"
#include "cmsis_os2.h"
#include "utils.h"
#include "zp_error.h"

template <typename T>
class MessageQueue : public IMessageQueue<T> {
private:
    osMessageQueueId_t const *queueId;

public:
    MessageQueue(osMessageQueueId_t *queueId) : queueId{queueId} {
        // blank
    }

    ZP_ERROR_e get(T *message) override {
        if (message == nullptr) return ZP_ERROR_NULLPTR;
        osStatus_t status = osMessageQueueGet(*queueId, message, 0, timeToTicks(100));
        return (status == osOK) ? ZP_ERROR_OK : ZP_ERROR_FAIL;
    }

    ZP_ERROR_e push(T *message) override {
        if (message == nullptr) return ZP_ERROR_NULLPTR;
        osStatus_t status = osMessageQueuePut(*queueId, message, 0, timeToTicks(100));
        return (status == osOK) ? ZP_ERROR_OK : ZP_ERROR_FAIL;
    }

    ZP_ERROR_e count(int &count_value) override {
        count_value = static_cast<int>(osMessageQueueGetCount(*queueId));
        return ZP_ERROR_OK;
    }

    ZP_ERROR_e remainingCapacity(int &capacity) override {
        capacity = static_cast<int>(osMessageQueueGetSpace(*queueId));
        return ZP_ERROR_OK;
    }
};