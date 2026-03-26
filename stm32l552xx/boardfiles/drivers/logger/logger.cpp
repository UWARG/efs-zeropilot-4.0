#include <cstdio>
#include <cstring>
#include "logger.hpp"

// Helper function to map FatFS FRESULT to ZP_ERROR_e
static ZP_ERROR_e mapFatFsError(FRESULT fresult) {
    switch (fresult) {
        case FR_OK:
            return ZP_ERROR_OK;
        case FR_DISK_ERR:
        case FR_INT_ERR:
            return ZP_ERROR_FAIL;
        case FR_NOT_READY:
            return ZP_ERROR_NOT_READY;
        case FR_NO_FILE:
        case FR_NO_PATH:
        case FR_NO_FILESYSTEM:
            return ZP_ERROR_RESOURCE_UNAVAILABLE;
        case FR_INVALID_NAME:
        case FR_INVALID_PARAMETER:
        case FR_INVALID_OBJECT:
        case FR_INVALID_DRIVE:
            return ZP_ERROR_INVALID_PARAM;
        case FR_DENIED:
        case FR_EXIST:
        case FR_WRITE_PROTECTED:
            return ZP_ERROR_UNSUPPORTED;
        case FR_TIMEOUT:
        case FR_LOCKED:
            return ZP_ERROR_TIMEOUT;
        case FR_NOT_ENOUGH_CORE:
            return ZP_ERROR_OUT_OF_MEMORY;
        case FR_TOO_MANY_OPEN_FILES:
            return ZP_ERROR_RESOURCE_UNAVAILABLE;
        default:
            return ZP_ERROR_FAIL;
    }
}

ZP_ERROR_e Logger::init() {
#if defined(SD_CARD_LOGGING)
    HAL_Delay(1000);

    FRESULT res;

    res = f_mount(&FatFs, "", 1);
    if (res != FR_OK) {
        return mapFatFsError(res);
    }

    FRESULT exist = FR_OK;
    FILINFO fno;
    int count = 1;

    while (exist == FR_OK) {
        snprintf(file, 100, "log%d.txt", count);
        exist = f_stat(file, &fno);
        count++;
    }

    return ZP_ERROR_OK;
#elif defined(SWO_LOGGING)
    return ZP_ERROR_OK;
#endif
}

ZP_ERROR_e Logger::log(const char message[100]) {
    char msgToSend[112]; //10 for timestamp, 100 for message, 2 for new line

    uint32_t ts = (uint32_t)(osKernelGetTickCount() * 1.0 / osKernelGetTickFreq());
    int tsStrLen = snprintf(msgToSend, 10, "%lus: ", ts);

#if defined(SD_CARD_LOGGING)
    FRESULT res;
    res = f_open(&fil, file, FA_WRITE | FA_OPEN_APPEND);
    if (res != FR_OK) {
        return mapFatFsError(res);
    }

    snprintf(msgToSend + tsStrLen, 100, message);
    snprintf(msgToSend + tsStrLen + strlen(message), 3, "\r\n");

    int putResult = f_puts(msgToSend, &fil);
    if (putResult < 0) {
        f_close(&fil);
        return ZP_ERROR_FAIL;
    }

    res = f_close(&fil);
    if (res != FR_OK) {
        return mapFatFsError(res);
    }

    return ZP_ERROR_OK;
#elif defined(SWO_LOGGING)
    snprintf(msgToSend + tsStrLen, 100, message);
    snprintf(msgToSend + tsStrLen + strlen(message), 3, "\r\n");
    printf("%s", msgToSend);
    return ZP_ERROR_OK;
#endif
}

ZP_ERROR_e Logger::log(const char message[][100], int count) {
    char msgToSend[112]; //10 for timestamp, 100 for message, 2 for new line

    uint32_t ts = (uint32_t)(osKernelGetTickCount() * 1.0 / osKernelGetTickFreq());
    int tsStrLen = snprintf(msgToSend, 10, "%lus: ", ts);

#if defined(SD_CARD_LOGGING)
    FRESULT res;
    res = f_open(&fil, file, FA_WRITE | FA_OPEN_APPEND);
    if (res != FR_OK) {
        return mapFatFsError(res);
    }

    for (int i = 0; i < count; i++) {
      snprintf(msgToSend + tsStrLen, 100, message[i]);
      snprintf(msgToSend + tsStrLen + strlen(message[i]), 3, "\r\n");

      int putResult = f_puts(msgToSend, &fil);
      if (putResult < 0) {
          f_close(&fil);
          return ZP_ERROR_FAIL;
      }
    }

    res = f_close(&fil);
    if (res != FR_OK) {
        return mapFatFsError(res);
    }

    return ZP_ERROR_OK;
#elif defined(SWO_LOGGING)
    for (int i = 0; i < count; i++) {
      snprintf(msgToSend + tsStrLen, 100, message[i]);
      snprintf(msgToSend + tsStrLen + strlen(message[i]), 3, "\r\n");
      printf("%s", msgToSend);
    }
    return ZP_ERROR_OK;
#endif
}
