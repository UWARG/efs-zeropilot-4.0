#pragma once
#include "logger_iface.hpp"
#include <fstream>
#include <iostream>

// Platform-specific directory creation
#ifdef _WIN32
    #include <direct.h>
    #define PLATFORM_MKDIR(path) _mkdir(path)
#else
    #include <sys/stat.h>
    #define PLATFORM_MKDIR(path) mkdir(path, 0755)
#endif

class SITL_Logger : public ILogger {
private:
    std::ofstream logFile;
    
public:
    SITL_Logger(const char* filename = "sd_card/sitl_log.txt") {
        // Create directory if it doesn't exist
        if (PLATFORM_MKDIR("sd_card") == 0) {
            std::cout << "[SITL_Logger] Created directory: sd_card" << std::endl;
        }
        
        logFile.open(filename, std::ios::app);
        
        if (!logFile.is_open()) {
            std::cerr << "[SITL_Logger] ERROR: Could not open log file: " << filename << std::endl;
        }
    }
    
    ~SITL_Logger() {
        if (logFile.is_open()) {
            logFile.close();
        }
    }
    
    int log(const char message[100]) override {
        if (logFile.is_open()) {
            logFile << message << std::endl;
            logFile.flush();
            return 0;
        }
        return -1;
    }
    
    int log(const char message[][100], int count) override {
        if (logFile.is_open()) {
            for (int i = 0; i < count; i++) {
                logFile << message[i] << std::endl;
            }
            logFile.flush();
            return 0;
        }
        return -1;
    }
};
