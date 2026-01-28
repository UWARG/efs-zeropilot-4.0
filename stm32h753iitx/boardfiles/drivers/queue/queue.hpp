#pragma once

#include "queue_iface.hpp"
#include "cmsis_os2.h"
#include "utils.h"

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
       * @retval status code
       */
      int get(T *message) override {
         return osMessageQueueGet(*queueId, message, 0, timeToTicks(100));
      }

      /**
       * @brief pushes message to the back of the queue
       * @param message data to be transmitted
       * @retval status code
       */
      int push(T *message) override {
         return osMessageQueuePut(*queueId, message, 0, timeToTicks(100));
      }

      /**
       * @brief returns the number of messages in the queue
       */
      int count() override {
         return osMessageQueueGetCount(*queueId);
      }

      /**
       * @brief Returns remaining space left in the queue
       * @retval number of available slots for messages
       */
      int remainingCapacity() override {
         return osMessageQueueGetSpace(*queueId);
      }
};