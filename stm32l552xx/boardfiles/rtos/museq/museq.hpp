#pragma once

#include "cmsis_os.h"

/* --- mutexes --- */
/* declare mutexes begin*/
extern osMutexId_t itmMutex;
/* declare mutexes end*/

void initMutexes();

/* --- semaphores --- */
/* declare semaphores begin */
/* declare semaphores end */

void initSemphrs();

/* --- queues --- */
/* declare queues begin */
extern osMessageQueueId_t amQueueId;
extern osMessageQueueId_t smLoggerQueueId;
extern osMessageQueueId_t smConfigAttitudeQueueId;
extern osMessageQueueId_t tmQueueId;
extern osMessageQueueId_t tmSmQueueId;
extern osMessageQueueId_t messageBufferId;
/* declare queues end */

void initQueues();
