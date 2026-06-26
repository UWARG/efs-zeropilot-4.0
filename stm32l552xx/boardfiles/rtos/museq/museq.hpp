#pragma once

#include "cmsis_os.h"
#include "filesystem_iface.hpp"

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
extern osMessageQueueId_t tmQueueId;
extern osMessageQueueId_t messageBufferId;
extern osMessageQueueId_t sdRequestQueueId;
extern osMessageQueueId_t sdBufferQueueId;
extern osMessageQueueId_t sdResponseQueueId[static_cast<size_t>(ManId::COUNT)];
/* declare queues end */

void initQueues();
