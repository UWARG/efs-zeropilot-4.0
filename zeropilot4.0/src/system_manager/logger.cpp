#include "logger.hpp"

namespace Logger {
    static constexpr const char* LOG_FILE = "logs/system.log";
    static constexpr size_t BUFFER_SIZE = 256;
    static constexpr uint32_t SYNC_PERIOD = 400; // 400ms, write_with_sync takes 40ms
    static IFileSystem* fileSystem = nullptr;
    static ISystemUtils* systemUtils = nullptr;
    static File logFile = {};
    static uint32_t lastSyncCount = 0;
    static uint32_t lastSyncTime = 0;
    static bool newWrite = false;

    void init(IFileSystem* fs, ISystemUtils* sysUtils) {
        fileSystem = fs;
        systemUtils = sysUtils;
        lastSyncCount = 0;
        lastSyncTime = 0;
        newWrite = false;
        if (!fileSystem) return;
        if (!fileSystem->available()) return;
        
        fileSystem->mkdir("logs");
        logFile = {};
        
        // Find first available log file (0-255), default to 0 if all taken
        char filename[32];
        FileInfo_t fno;
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

    
    void log(const char* format, LogLevel_e level, ...) {
        if (!fileSystem || !systemUtils) return;
        
        char buffer[BUFFER_SIZE];
        
        // Add timestamp
        uint32_t ts = systemUtils->getCurrentTimestampMs() / 1000;
        int tsLen = snprintf(buffer, 10, "%us ", ts);
        
        // Add log level
        const char* levelStr = "";
        switch (level) {
            case LogLevel_e::LOG_DEBUG: levelStr = "DEBUG"; break;
            case LogLevel_e::LOG_INFO: levelStr = "INFO"; break;
            case LogLevel_e::LOG_WARN: levelStr = "WARN"; break;
            case LogLevel_e::LOG_CRITICAL: levelStr = "CRITICAL"; break;
        }
        int levelLen = snprintf(buffer + tsLen, BUFFER_SIZE - tsLen - 1, "[%s] ", levelStr);
        
        // Add formatted message
        va_list args;
        va_start(args, level);
        int msgLen = vsnprintf(buffer + tsLen + levelLen, BUFFER_SIZE - tsLen - levelLen - 1, format, args);
        va_end(args);
        
        int totalLen = tsLen + levelLen + msgLen;
        if (totalLen > BUFFER_SIZE - 2) {
            totalLen = BUFFER_SIZE - 2; // Truncate if message exceeds buffer
        }
        buffer[totalLen] = '\n';
        buffer[totalLen + 1] = '\0';
        
        // fileSystem->writeAndSync(ManId_e::SYSTEM, &logFile, buffer, totalLen + 2, ReqOptions_e::ASYNC_NO_RESP);
        if (level == LogLevel_e::LOG_CRITICAL || lastSyncCount >= 10 ) { // Sync every sync period, every 10 writes, or immediately for critical logs
            fileSystem->writeAndSync(ManId_e::SYSTEM, &logFile, buffer, totalLen + 2, ReqOptions_e::ASYNC_NO_RESP);
            lastSyncCount = 0;
        } else {
            fileSystem->write(ManId_e::SYSTEM, &logFile, buffer, totalLen + 2, nullptr, ReqOptions_e::ASYNC_NO_RESP);
            lastSyncCount++;
        }
        newWrite = true;
    }
    
    void sync() {
        if (fileSystem && newWrite && systemUtils->getCurrentTimestampMs() - lastSyncTime >= SYNC_PERIOD) {
            fileSystem->sync(ManId_e::SYSTEM, &logFile, ReqOptions_e::ASYNC_NO_RESP);
            lastSyncTime = systemUtils->getCurrentTimestampMs();
            newWrite = false;
        }
    }

    /* TODO: Verify in later PR
    void shutdown() {
        if (fileSystem) {
            fileSystem->close(&logFile);
        }
        fileSystem = nullptr;
        systemUtils = nullptr;
    }
    */

} // namespace Logger
