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
    
    // Find first available log file (0-255), default to 0 if all taken
    char filename[32];
    FileInfo fno;
    uint32_t fileNum = 0;
    for (uint32_t i = 0; i < 255; i++) {
        snprintf(filename, sizeof(filename), "logs/system%u.log", i);
        if (fileSystem->stat(filename, &fno) != FILE_STATUS_OK) {
            fileNum = i;
            break;
        }
    }
    
    snprintf(filename, sizeof(filename), "logs/system%u.log", fileNum);
    fileSystem->open(&logFile, filename, "a");
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
    int tsLen = snprintf(buffer, 10, "[%us] ", ts);
    
    // Add formatted message
    va_list args;
    va_start(args, format);
    int msgLen = vsnprintf(buffer + tsLen, BUFFER_SIZE - tsLen - 1, format, args);
    va_end(args);
    
    buffer[tsLen + msgLen] = '\0';
    
    fileSystem->printf(&logFile, "%s\n", buffer);
    fileSystem->sync(&logFile);
}
