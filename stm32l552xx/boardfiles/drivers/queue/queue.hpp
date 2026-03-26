#pragma once

#include "queue_iface.hpp"
#include "cmsis_os2.h"
#include "utils.h"
#include "error.h"

template <typename T>
class MessageQueue : public IMessageQueue<T> {
   private:
      osMessageQueueId_t * const queueId;

   public:
      MessageQueue(osMessageQueueId_t *queueId) : queueId{queueId} {
        // blank
      }

      /**
       * @brief Gets top element of queue
       * @param message variable to receive data
       * @retval ZP_ERROR_e status code
       */
      ZP_ERROR_e get(T *message) override {
         if (message == NULL) {
            return ZP_ERROR_NULLPTR;
         }
         if (queueId == NULL || *queueId == NULL) {
            return ZP_ERROR_NULLPTR;
         }

         uint32_t ticks;
         ZP_ERROR_e err = timeToTicks(&ticks, 100);
         if (err != ZP_ERROR_OK) {
            return err;
         }

         osStatus_t status = osMessageQueueGet(*queueId, message, 0, ticks);
         if (status == osOK) {
            return ZP_ERROR_OK;
         } else if (status == osErrorTimeout) {
            return ZP_ERROR_TIMEOUT;
         } else if (status == osErrorResource) {
            return ZP_ERROR_RESOURCE_UNAVAILABLE;
         }
         return ZP_ERROR_FAIL;
      }

      /**
       * @brief pushes message to the back of the queue
       * @param message data to be transmitted
       * @retval ZP_ERROR_e status code
       */
      ZP_ERROR_e push(T *message) override {
         if (message == NULL) {
            return ZP_ERROR_NULLPTR;
         }
         if (queueId == NULL || *queueId == NULL) {
            return ZP_ERROR_NULLPTR;
         }

         uint32_t ticks;
         ZP_ERROR_e err = timeToTicks(&ticks, 100);
         if (err != ZP_ERROR_OK) {
            return err;
         }

         osStatus_t status = osMessageQueuePut(*queueId, message, 0, ticks);
         if (status == osOK) {
            return ZP_ERROR_OK;
         } else if (status == osErrorTimeout) {
            return ZP_ERROR_TIMEOUT;
         } else if (status == osErrorResource) {
            return ZP_ERROR_MEMORY_OVERFLOW;
         }
         return ZP_ERROR_FAIL;
      }

      /**
       * @brief returns the number of messages in the queue
       */
      ZP_ERROR_e count(int *count_value) override {
         if (count_value == NULL) {
            return ZP_ERROR_NULLPTR;
         }
         if (queueId == NULL || *queueId == NULL) {
            return ZP_ERROR_NULLPTR;
         }

         *count_value = osMessageQueueGetCount(*queueId);
         return ZP_ERROR_OK;
      }

      /**
       * @brief Returns remaining space left in the queue
       * @retval number of available slots for messages
       */
      ZP_ERROR_e remainingCapacity(int *capacity) override {
         if (capacity == NULL) {
            return ZP_ERROR_NULLPTR;
         }
         if (queueId == NULL || *queueId == NULL) {
            return ZP_ERROR_NULLPTR;
         }

         *capacity = osMessageQueueGetSpace(*queueId);
         return ZP_ERROR_OK;
      }
};