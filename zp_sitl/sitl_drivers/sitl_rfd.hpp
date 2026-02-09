#pragma once
#include "rfd_iface.hpp"
#include <cstring>

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

class SITL_RFD : public IRFD {
private:
#ifdef _WIN32
    SOCKET sockfd; // Windows uses a specific SOCKET type
#else
    int sockfd;
#endif
    struct sockaddr_in destAddr;
    
public:
    SITL_RFD(const char* ip = "127.0.0.1", int port = 14550) {
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
#else
        if (sockfd < 0) return;
#endif
        
        memset(&destAddr, 0, sizeof(destAddr));
        destAddr.sin_family = AF_INET;
        destAddr.sin_port = htons(port);
        inet_pton(AF_INET, ip, &destAddr.sin_addr);
    }
    
    ~SITL_RFD() {
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
        }
    }
    
    uint16_t receive(uint8_t* buffer, uint16_t bufferSize) override {
        return 0;
    }
};
