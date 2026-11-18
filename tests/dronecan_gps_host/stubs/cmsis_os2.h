#pragma once
#include <cstdint>
using osMutexId_t = void*;
using osStatus_t = int;
constexpr int osOK=0;
extern bool lockBusy, allocationFails;
inline void* osMutexNew(void*) { return allocationFails ? nullptr : reinterpret_cast<void*>(1); }
inline int osMutexAcquire(void*, unsigned) { return lockBusy ? 1 : 0; }
inline int osMutexRelease(void*) { return 0; }
extern uint32_t nowMs;
inline uint32_t HAL_GetTick() { return nowMs; }
