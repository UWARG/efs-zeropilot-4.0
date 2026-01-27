#pragma once

#ifdef __cplusplus

#include "startup_threads.hpp"
#include "am_threads.hpp"
#include "sm_threads.hpp"
#include "tm_threads.hpp"

#endif

#include "error.h"

#ifdef __cplusplus
extern "C" {
#endif

    ZP_ERROR_e initThreads();

#ifdef __cplusplus
}
#endif