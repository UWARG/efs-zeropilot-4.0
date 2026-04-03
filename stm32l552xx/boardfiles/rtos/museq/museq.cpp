#include "museq.hpp"
#include "rc_motor_control.hpp"
#include "tm_queue.hpp"
#include "sd_fatfs_msgs.hpp"
#include "mavlink.h"

/* --- mutexes --- */
/* define mutexes begin */
osMutexId_t itmMutex;
osMessageQueueId_t amQueueId;
osMessageQueueId_t tmQueueId;
osMessageQueueId_t messageBufferId;
osMessageQueueId_t sdRequestQueueId;
osMessageQueueId_t sdBufferQueueId;
osMessageQueueId_t sdResponseQueueId[static_cast<size_t>(ManId::COUNT)];

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
  sdRequestQueueId = osMessageQueueNew(32, sizeof(FatFSReqMsg), NULL);
  sdBufferQueueId = osMessageQueueNew(32, sizeof(FatFSReqBuff), NULL);
  for (int i = 0; i < static_cast<int>(ManId::COUNT); ++i) {
      sdResponseQueueId[i] = osMessageQueueNew(16, sizeof(PollResult), NULL);
  }
}
