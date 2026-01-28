#include "logger.hpp"
#include "config_utils/config_defines.hpp"
#include <cstdio>
#include <cstdint>
#include <cstring>

Logger::Logger(ITextIO *textIO, ISystemUtils *sysUtils) : textIO(textIO), sysUtils(sysUtils) {
  //blank
}

int Logger::init() {
#if defined(SD_CARD_LOGGING)
    sysUtils->delayMs(1000); //wait for SD card to stabilize

    int res;

    res = textIO->mountFile();
    if (res != 0) {
        return res;
    }

    int exist = 0;
    int count = 1;

    while (exist == 0) {
        snprintf(logFile, 100, "log%d.txt", count);
        exist = textIO->checkFileExist(logFile);
        count++;
    }

    return res;
#elif defined(SWO_LOGGING)
    return 0;
#endif

}

int Logger::log(const char message[100]) {
    char msgToSend[112]; //10 for timestamp, 100 for message, 2 for new line

    uint32_t ts = sysUtils->getCurrentTimestampMs() / 1000;
    int tsStrLen = snprintf(msgToSend, 10, "%us: ", ts);

#if defined(SD_CARD_LOGGING)
    int res;
    res = textIO->open(logFile, FA_WRITE | FA_OPEN_APPEND);

    snprintf(msgToSend + tsStrLen, 100, message);
    snprintf(msgToSend + tsStrLen + strlen(message), 3, "\r\n");
    res = textIO->write(msgToSend);

    res = textIO->close();

    return res;
#elif defined(SWO_LOGGING)
    snprintf(msgToSend + tsStrLen, 100, message);
    snprintf(msgToSend + tsStrLen + strlen(message), 3, "\r\n");
    printf("%s", msgToSend);
    return 0;
#endif
}

int Logger::log(const char message[][100], int count) {
    char msgToSend[112]; //10 for timestamp, 100 for message, 2 for new line

    uint32_t ts = sysUtils->getCurrentTimestampMs() / 1000;
    int tsStrLen = snprintf(msgToSend, 10, "%us: ", ts);

#if defined(SD_CARD_LOGGING)
    int res;
    textIO->open(logFile, FA_WRITE | FA_OPEN_APPEND);

    for (int i = 0; i < count; i++) {
      snprintf(msgToSend + tsStrLen, 100, message[i]);
      snprintf(msgToSend + tsStrLen + strlen(message[i]), 3, "\r\n");
      res = textIO->write(msgToSend);
    }

    res = textIO->close();

    return res;
#elif defined(SWO_LOGGING)
    for (int i = 0; i < count; i++) {
      snprintf(msgToSend + tsStrLen, 100, message[i]);
      snprintf(msgToSend + tsStrLen + strlen(message[i]), 3, "\r\n");
      printf("%s", msgToSend);
    }
    return 0;
#endif
}
