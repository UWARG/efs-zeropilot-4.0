#include "logger.hpp"

namespace Logger {
    static constexpr const char* LOG_FILE = "logs/system.log";
    static constexpr size_t BUFFER_SIZE = 256;
    static IFileSystem* fileSystem = nullptr;
    static ISystemUtils* systemUtils = nullptr;
    static File logFile = {};
    static int lastSync = 0;

    void Logger::init(IFileSystem* fs, ISystemUtils* sysUtils) {
        fileSystem = fs;
        systemUtils = sysUtils;
        if (!fileSystem) return;
        if (!fileSystem->available()) return;
        
        fileSystem->mkdir(ManId::SYSTEM, "logs");
        while (fileSystem->poll(ManId::SYSTEM) == FILE_STATUS_NOT_DONE) {
            systemUtils->delayMs(10);
        }
        logFile = {};
        
        // Find first available log file (0-255), default to 0 if all taken
        char filename[32];
        FileInfo fno;
        uint32_t fileNum = 0;
        for (uint32_t i = 0; i < 255; i++) {
            snprintf(filename, sizeof(filename), "logs/system%u.log", i);
            if (fileSystem->stat(ManId::SYSTEM, filename, &fno) != FILE_STATUS_OK) {
                fileNum = i;
                break;
            }
        }
        
        snprintf(filename, sizeof(filename), "logs/system%u.log", fileNum);
        fileSystem->open(ManId::SYSTEM, &logFile, filename, "a");
    }

    void Logger::shutdown() {
        if (fileSystem) {
            fileSystem->close(ManId::SYSTEM, &logFile);
        }
        fileSystem = nullptr;
        systemUtils = nullptr;
    }

    void Logger::log(const char* format, LogLevel level, ...) {
        if (!fileSystem || !systemUtils) return;
        
        char buffer[BUFFER_SIZE];
        
        // Add timestamp
        uint32_t ts = systemUtils->getCurrentTimestampMs() / 1000;
        int tsLen = snprintf(buffer, 10, "%us ", ts);

        // Add log level
        const char* levelStr = "";
        switch (level) {
            case LogLevel::DEBUG: levelStr = "DEBUG"; break;
            case LogLevel::INFO: levelStr = "INFO"; break;
            case LogLevel::WARN: levelStr = "WARN"; break;
            case LogLevel::CRITICAL: levelStr = "CRITICAL"; break;
        }
        int levelLen = snprintf(buffer + tsLen, BUFFER_SIZE - tsLen - 1, "[%s] ", levelStr);
        
        // Add formatted message
        va_list args;
        va_start(args, format);
        int msgLen = vsnprintf(buffer + tsLen + levelLen, BUFFER_SIZE - tsLen - levelLen - 1, format, args);
        va_end(args);
        
        int totalLen = tsLen + levelLen + msgLen;
        if (totalLen > BUFFER_SIZE - 2) {
            totalLen = BUFFER_SIZE - 2; // Truncate if message exceeds buffer
        }
        buffer[totalLen] = '\n';
        buffer[totalLen + 1] = '\0';
        
        if (fileSystem->available()) {
            if (level == LogLevel::CRITICAL || lastSync >= 10) { // Sync every 10 writes automatically, or immediately for critical logs
                fileSystem->write_and_sync(ManId::SYSTEM, &logFile, buffer, totalLen + 2, nullptr);
            } else {
                fileSystem->write(ManId::SYSTEM, &logFile, buffer, totalLen + 2, nullptr);
                lastSync++;
            }
        }
    }

    void Logger::sync() {
        if (fileSystem) {
            fileSystem->sync(ManId::SYSTEM, &logFile);
        }
    }
} // namespace Logger
