#pragma once

#include "file_system_iface.hpp"
#include "systemutils_iface.hpp"
#include <cstdarg>
#include <cstdio>

class Logger {
private:
    static constexpr const char* LOG_FILE = "logs/system.log";
    static constexpr size_t BUFFER_SIZE = 256;
    static IFileSystem* fileSystem;
    static ISystemUtils* systemUtils;
    static File logFile;
    
public:
    static void init(IFileSystem* fs, ISystemUtils* sysUtils);
    static void shutdown();
    static void log(const char* format, ...);
};
