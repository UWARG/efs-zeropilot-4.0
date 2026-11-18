#pragma once
#include <ctime>
// Host-only bridge; STM32 newlib provides gmtime_r natively.
inline tm* gps_test_gmtime_r(const time_t* stamp, tm* out) { return gmtime_s(out, stamp)==0 ? out : nullptr; }
#define gmtime_r gps_test_gmtime_r
