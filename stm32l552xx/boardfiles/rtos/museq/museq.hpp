#pragma once

#include "cmsis_os.h"
#include "error.h"

/* --- mutexes --- */
/* declare mutexes begin*/
extern osMutexId_t itmMutex;
/* declare mutexes end*/

#ifdef __cplusplus
extern "C" {
#endif

    ZP_ERROR_e initMutexes();

    /* --- semaphores --- */
    /* declare semaphores begin */
    /* declare semaphores end */

    ZP_ERROR_e initSemphrs();

    /* --- queues --- */
    /* declare queues begin */
    extern osMessageQueueId_t amQueueId;
    extern osMessageQueueId_t smLoggerQueueId;
    extern osMessageQueueId_t tmQueueId;
    extern osMessageQueueId_t messageBufferId;
    /* declare queues end */

    ZP_ERROR_e initQueues();

#ifdef __cplusplus
}
#endif
