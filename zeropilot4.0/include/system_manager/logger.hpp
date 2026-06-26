#pragma once

#include "filesystem_iface.hpp"
#include "systemutils_iface.hpp"
#include <cstdarg>
#include <cstdio>

enum class LogLevel : uint8_t {
    LOG_DEBUG = 0,
    LOG_INFO,
    LOG_WARN,
    LOG_CRITICAL
};

namespace Logger {
    void init(IFileSystem* fs, ISystemUtils* sysUtils);
    void shutdown();
    void log(const char* format, LogLevel level, ...);
    void sync();
};
