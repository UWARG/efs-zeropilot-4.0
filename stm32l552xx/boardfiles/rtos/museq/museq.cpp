#include "museq.hpp"
#include "rc_motor_control.hpp"
#include "tm_queue.hpp"
#include "mavlink.h"
#include "error.h"

/* --- mutexes --- */
/* define mutexes begin */
osMutexId_t itmMutex;
osMessageQueueId_t amQueueId;
osMessageQueueId_t smLoggerQueueId;
osMessageQueueId_t tmQueueId;
osMessageQueueId_t messageBufferId;

static const osMutexAttr_t itmMutexAttr = {
  "itmMutex",
  osMutexPrioInherit,
  NULL,
  0U
};
/* define mutexes end */

extern "C" {
  ZP_ERROR_e initMutexes()
  {
    itmMutex = osMutexNew(&itmMutexAttr);

    if (itmMutex == NULL) {
      return ZP_ERROR_FAIL;
    }

    return ZP_ERROR_OK;
  }

  /* --- sempahores --- */
  /* define semaphores begin */
  /* define semaphores end */

  ZP_ERROR_e initSemphrs()
  {
    return ZP_ERROR_OK;
  }

  /* --- queues --- */
  /* define queues begin */
  /* define queues end */

  ZP_ERROR_e initQueues()
  {
    amQueueId = osMessageQueueNew(16, sizeof(RCMotorControlMessage_t), NULL);
    if (amQueueId == NULL) {
      return ZP_ERROR_FAIL;
    }

    smLoggerQueueId = osMessageQueueNew(16, sizeof(char[100]), NULL);
    if (smLoggerQueueId == NULL) {
      return ZP_ERROR_FAIL;
    }

    tmQueueId = osMessageQueueNew(16, sizeof(TMMessage_t), NULL);
    if (tmQueueId == NULL) {
      return ZP_ERROR_FAIL;
    }

    messageBufferId = osMessageQueueNew(16, sizeof(mavlink_message_t), NULL);
    if (messageBufferId == NULL) {
      return ZP_ERROR_FAIL;
    }

    return ZP_ERROR_OK;
  }
}
