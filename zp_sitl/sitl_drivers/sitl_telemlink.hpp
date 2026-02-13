#pragma once
#include "telemlink_iface.hpp"
#include <cstring>
#include <functional>
#include <string>
#include <sstream>
#include <iomanip>
#include <iostream>

#ifdef _WIN32
    #ifndef WIN32_LEAN_AND_MEAN
    #define WIN32_LEAN_AND_MEAN
    #endif
    #include <winsock2.h>
    #include <ws2tcpip.h>
    #include <io.h>
    // Link the networking library automatically for MSVC
    #pragma comment(lib, "ws2_32.lib")
    typedef int socklen_t;
#else
    #include <sys/socket.h>
    #include <netinet/in.h>
    #include <arpa/inet.h>
    #include <unistd.h>
    #include <fcntl.h>
#endif

class SITL_TELEM : public ITelemLink {
private:
    using Config = SITL_Driver_Configs::SITL_TELEM_Config;
#ifdef _WIN32
    SOCKET sockfd; // Windows uses a specific SOCKET type
#else
    int sockfd;
#endif
    struct sockaddr_in destAddr;
    std::function<void(const std::string&, uint8_t)> telemLogCallback;

public:
    SITL_TELEM(const char* ip, int port, std::function<void(const std::string&, uint8_t)> telemLogCallback = nullptr)
        : telemLogCallback(telemLogCallback) {
#ifdef _WIN32
        // Initialize Windows Sockets (Required on Windows)
        WSADATA wsaData;
        if (WSAStartup(MAKEWORD(2, 2), &wsaData) != 0) {
            sockfd = INVALID_SOCKET;
            return;
        }
#endif
        sockfd = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
        
#ifdef _WIN32
        if (sockfd == INVALID_SOCKET) return;
        // Set non-blocking mode on Windows
        u_long mode = 1;
        ioctlsocket(sockfd, FIONBIO, &mode);
        // Increase receive buffer on Windows
        int rcvbuf = Config::RX_BUF_SZ_BYTES;
        setsockopt(sockfd, SOL_SOCKET, SO_RCVBUF, (const char*)&rcvbuf, sizeof(rcvbuf));
#else
        if (sockfd < 0) return;
        // Set non-blocking mode on Unix
        fcntl(sockfd, F_SETFL, O_NONBLOCK);
        // Increase receive buffer on Unix
        int rcvbuf = Config::RX_BUF_SZ_BYTES;
        setsockopt(sockfd, SOL_SOCKET, SO_RCVBUF, (void*)&rcvbuf, sizeof(rcvbuf));
#endif
        
        memset(&destAddr, 0, sizeof(destAddr));
        destAddr.sin_family = AF_INET;
        destAddr.sin_port = htons(port);
        inet_pton(AF_INET, ip, &destAddr.sin_addr);
    }
    
    ~SITL_TELEM() {
#ifdef _WIN32
        if (sockfd != INVALID_SOCKET) {
            closesocket(sockfd); // Windows uses closesocket()
            WSACleanup();        // Clean up networking stack
        }
#else
        if (sockfd >= 0) close(sockfd);
#endif
    }
    
    void transmit(const uint8_t* data, uint16_t size) override {
#ifdef _WIN32
        if (sockfd != INVALID_SOCKET) {
#else
        if (sockfd >= 0) {
#endif
            sendto(sockfd, (const char*)data, size, 0, (struct sockaddr*)&destAddr, sizeof(destAddr));
            if (telemLogCallback) {
                std::ostringstream oss;
                for (uint16_t i = 0; i < size; ++i) {
                    oss << std::hex << std::uppercase << std::setw(2) << std::setfill('0') << (int)data[i];
                    if (i < size - 1) {
                        oss << " "; // Add space between bytes
                    }
                }
                oss << "\n"; // Add newline for clarity
                telemLogCallback(oss.str(), 1);
            }
        }
    }
    
    uint16_t receive(uint8_t* buffer, uint16_t bufferSize) override {
#ifdef _WIN32
        if (sockfd != INVALID_SOCKET) {
#else
        if (sockfd >= 0) {
#endif
            struct sockaddr_in srcAddr;
            socklen_t addrLen = sizeof(srcAddr);
            int receivedBytes = recvfrom(sockfd, (char*)buffer, bufferSize, 0, (struct sockaddr*)&srcAddr, &addrLen);
            
            if (receivedBytes > 0) {
                if (telemLogCallback) {
                    std::ostringstream oss;
                    for (int i = 0; i < receivedBytes; ++i) {
                        oss << std::hex << std::uppercase << std::setw(2) << std::setfill('0') << (int)buffer[i];
                        if (i < receivedBytes - 1) {
                            oss << " ";
                        }
                    }
                    oss << "\n";
                    telemLogCallback(oss.str(), 0);
                }
                return static_cast<uint16_t>(receivedBytes);
            }
        }
        return 0;
    }
};
