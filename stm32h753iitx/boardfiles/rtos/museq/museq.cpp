#include "museq.hpp"
#include "rc_motor_control.hpp"
#include "tm_queue.hpp"
#include "exmem_msgs.hpp"
#include "mavlink.h"

/* --- mutexes --- */
/* define mutexes begin */
osMutexId_t itmMutex;
osMessageQueueId_t amQueueId;
osMessageQueueId_t tmQueueId;
osMessageQueueId_t messageBufferId;
osMessageQueueId_t sdRequestQueueId;
osMessageQueueId_t sdBufferQueueId;
osMessageQueueId_t sdResponseQueueId[static_cast<size_t>(ManagerId_e::NUM_MANAGERS)];

static const osMutexAttr_t itmMutexAttr = {
  "itmMutex",
  osMutexPrioInherit,
  NULL,
  0U
};
/* define mutexes end */

void initMutexes()
{
  itmMutex = osMutexNew(&itmMutexAttr);
}

/* --- sempahores --- */
/* define semaphores begin */
/* define semaphores end */

void initSemphrs()
{

}

/* --- queues --- */
/* define queues begin */
/* define queues end */

void initQueues()
{
  amQueueId = osMessageQueueNew(16, sizeof(RCMotorControlMessage_t), NULL);
  tmQueueId = osMessageQueueNew(16, sizeof(TMMessage_t), NULL);
  messageBufferId = osMessageQueueNew(16, sizeof(mavlink_message_t), NULL);
  sdRequestQueueId = osMessageQueueNew(32, sizeof(ExMemReqMsg), NULL);
  sdBufferQueueId = osMessageQueueNew(32, sizeof(ExMemReqBuf), NULL);
  for (int i = 0; i < static_cast<int>(ManagerId_e::NUM_MANAGERS); ++i) {
      sdResponseQueueId[i] = osMessageQueueNew(16, sizeof(PollResult), NULL);
  }
}
