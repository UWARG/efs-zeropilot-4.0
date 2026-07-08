#pragma once

#include "cmsis_os.h"
#include "gps_datatypes.hpp"
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

/* --- queues --- */
/* declare queues begin */
extern osMessageQueueId_t amQueueId;
extern osMessageQueueId_t smLoggerQueueId;
extern osMessageQueueId_t tmQueueId;
extern osMessageQueueId_t messageBufferId;
/* declare queues end */

void initQueues();

extern rtcm_correction_data_t sharedRtcmBuffer;