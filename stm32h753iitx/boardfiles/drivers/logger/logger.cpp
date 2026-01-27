#include <cstdio>
#include <cstring>
#include "logger.hpp"

int Logger::init() {
#if defined(SD_CARD_LOGGING)
    HAL_Delay(1000);

    FRESULT res;

    res = f_mount(&FatFs, "", 1);
    if (res != FR_OK) {
        return res;
    }

    FRESULT exist = FR_OK;
    FILINFO fno;
    int count = 1;

    while (exist == FR_OK) {
        snprintf(file, 100, "log%d.txt", count);
        exist = f_stat(file, &fno);
        count++;
    }

    return res;
#elif defined(SWO_LOGGING)
    return 0;
#endif
}

int Logger::log(const char message[100]) {
    char msgToSend[112]; //10 for timestamp, 100 for message, 2 for new line

    uint32_t ts = (uint32_t)(osKernelGetTickCount() * 1.0 / osKernelGetTickFreq());
    int tsStrLen = snprintf(msgToSend, 10, "%lus: ", ts);

#if defined(SD_CARD_LOGGING)
    FRESULT res;
    res = f_open(&fil, file, FA_WRITE | FA_OPEN_APPEND);

    snprintf(msgToSend + tsStrLen, 100, message);
    snprintf(msgToSend + tsStrLen + strlen(message), 3, "\r\n");
    res = (FRESULT)f_puts(msgToSend, &fil);

    res = f_close(&fil);

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

    uint32_t ts = (uint32_t)(osKernelGetTickCount() * 1.0 / osKernelGetTickFreq());
    int tsStrLen = snprintf(msgToSend, 10, "%lus: ", ts);

#if defined(SD_CARD_LOGGING)
    FRESULT res;
    res = f_open(&fil, file, FA_WRITE | FA_OPEN_APPEND);

    for (int i = 0; i < count; i++) {
      snprintf(msgToSend + tsStrLen, 100, message[i]);
      snprintf(msgToSend + tsStrLen + strlen(message[i]), 3, "\r\n");
      res = (FRESULT)f_puts(msgToSend, &fil);
    }

    res = f_close(&fil);

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
