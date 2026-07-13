#pragma once

#include "cmsis_os.h"
#include "gps_iface.hpp"

/* --- mutexes --- */
/* declare mutexes begin*/
extern osMutexId_t itmMutex;
/* declare mutexes end*/

void initMutexes();

/* --- semaphores --- */
/* declare semaphores begin */
/* declare semaphores end */

void initSemphrs();

extern rtcmCorrectionData_t sharedRtcmBuffer;

/* --- queues --- */
/* declare queues begin */
extern osMessageQueueId_t amQueueId;
extern osMessageQueueId_t smLoggerQueueId;
extern osMessageQueueId_t tmQueueId;
extern osMessageQueueId_t messageBufferId;
/* declare queues end */

void initQueues();
