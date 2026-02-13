#include "logger.hpp"

// Static member definitions
IFileSystem* Logger::fileSystem = nullptr;
ISystemUtils* Logger::systemUtils = nullptr;
File Logger::logFile = {};

void Logger::init(IFileSystem* fs, ISystemUtils* sysUtils) {
    fileSystem = fs;
    systemUtils = sysUtils;
    if (!fileSystem) return;
    
    fileSystem->mkdir("logs");
    logFile = {};
    fileSystem->open(&logFile, LOG_FILE, "a");
}

void Logger::shutdown() {
    if (fileSystem) {
        fileSystem->close(&logFile);
    }
    fileSystem = nullptr;
    systemUtils = nullptr;
}

void Logger::log(const char* format, ...) {
    if (!fileSystem || !systemUtils) return;
    
    char buffer[BUFFER_SIZE];
    
    // Add timestamp
    uint32_t ts = systemUtils->getCurrentTimestampMs() / 1000;
    int tsLen = snprintf(buffer, 16, "[%us] ", ts);
    
    // Add formatted message
    va_list args;
    va_start(args, format);
    int msgLen = vsnprintf(buffer + tsLen, BUFFER_SIZE - tsLen - 1, format, args);
    va_end(args);
    
    buffer[tsLen + msgLen] = '\0';
    
    fileSystem->printf(&logFile, "%s\n", buffer);
    fileSystem->sync(&logFile);
}
