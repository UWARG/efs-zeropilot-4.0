#include "logger.hpp"
#include <cstdio>
#include <cstdint>
#include <cstring>

Logger::Logger(ITextIO *textIO, ISystemUtils *sysUtils) : textIO(textIO), sysUtils(sysUtils) {
  //blank
}

int Logger::init() {
    sysUtils->delayMs(1000); //wait for SD card to stabilize

    int res;

    res = textIO->mountFile();
    if (res != 0) {
        return res;
    }

    bool exist = true;
    int count = 1;

    while (exist == true) {
        snprintf(logFile, 100, "log%d.txt", count);
        exist = textIO->checkFileExist(logFile);
        count++;
    }

    return res;
}

int Logger::log(const char message[100]) {
    char msgToSend[112]; //10 for timestamp, 100 for message, 2 for new line

    uint32_t ts = sysUtils->getCurrentTimestampMs() / 1000;
    int tsStrLen = snprintf(msgToSend, 10, "%lus: ", ts);

    int res;
    res = textIO->open(logFile, FA_WRITE | FA_OPEN_APPEND);

    snprintf(msgToSend + tsStrLen, 100, message);
    snprintf(msgToSend + tsStrLen + strlen(message), 3, "\r\n");
    res = textIO->write(msgToSend);

#if defined(DEBUG)
    printf("%s", msgToSend);
#endif

    res = textIO->close();

    return res;
}

int Logger::log(const char message[][100], int count) {
    char msgToSend[112]; //10 for timestamp, 100 for message, 2 for new line

    uint32_t ts = sysUtils->getCurrentTimestampMs() / 1000;
    int tsStrLen = snprintf(msgToSend, 10, "%lus: ", ts);

    int res;
    textIO->open(logFile, FA_WRITE | FA_OPEN_APPEND);

    for (int i = 0; i < count; i++) {
      snprintf(msgToSend + tsStrLen, 100, message[i]);
      snprintf(msgToSend + tsStrLen + strlen(message[i]), 3, "\r\n");
      res = textIO->write(msgToSend);

#if defined(DEBUG)
        printf("%s", msgToSend);
#endif
    }

    res = textIO->close();

    return res;
}