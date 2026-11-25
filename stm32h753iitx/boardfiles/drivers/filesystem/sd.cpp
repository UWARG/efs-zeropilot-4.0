#include "sd.hpp"
#include <new>
#include <cstring>
#include <cstdio>
#include <cstdarg>
#include "stm32h7xx_hal.h"

#ifdef SWO_LOGGING
extern "C" int _write(int file, char *ptr, int len);
static inline void swoWrite(const char* data, uint32_t len) {
    _write(0, const_cast<char*>(data), static_cast<int>(len));
}
#endif

SDFileSystem::SDFileSystem(MessageQueue<SdReqMsg> *reqQueue, MessageQueue<SdReqBuf> *bufQueue, IMessageQueue<PollResult> *respQueues[static_cast<size_t>(ManagerId_e::NUM_MANAGERS)])
    : mounted(false), requestQueue(reqQueue), bufferQueue(bufQueue), responseQueues(respQueues) {
    std::memset(&fsObj, 0, sizeof(FATFS));
}

SDFileSystem::~SDFileSystem() {
    if (mounted) {
        f_mount(nullptr, "", 0);
    }
}

ZP_Error SDFileSystem::fresultToZpError(FRESULT res) {
    switch (res) {
    case FR_OK: return ZP_ERROR_OK;
    case FR_DISK_ERR:
    case FR_INT_ERR:
    case FR_MKFS_ABORTED:
    	return ZP_ERROR_FAIL | ZP_ERROR_EXT_API;
    case FR_NOT_READY:
    	return ZP_ERROR_NOT_READY | ZP_ERROR_EXT_API;
    case FR_NO_FILE:
    case FR_NO_PATH:
    	return ZP_ERROR_FAIL | ZP_ERROR_EXT_API;
    case FR_INVALID_NAME:
    case FR_INVALID_OBJECT:
    case FR_INVALID_DRIVE:
    case FR_INVALID_PARAMETER:
    	return ZP_ERROR_INVALID_ARG | ZP_ERROR_EXT_API;
    case FR_DENIED:
    case FR_EXIST:
    case FR_WRITE_PROTECTED:
    case FR_LOCKED:
    	return ZP_ERROR_FAIL | ZP_ERROR_EXT_API;
    case FR_NOT_ENABLED:
    case FR_NO_FILESYSTEM:
    	return ZP_ERROR_CONFIG | ZP_ERROR_EXT_API;
    case FR_TIMEOUT:
    	return ZP_ERROR_TIMEOUT | ZP_ERROR_EXT_API;
    case FR_NOT_ENOUGH_CORE:
    	return ZP_ERROR_MEMORY_OVERFLOW | ZP_ERROR_EXT_API;
    case FR_TOO_MANY_OPEN_FILES:
    	return ZP_ERROR_BUSY | ZP_ERROR_EXT_API;
    default:
    	return ZP_ERROR_FAIL | ZP_ERROR_EXT_API;
    }
}

uint8_t SDFileSystem::modeStringToFatfsFlags(const char* mode) {
    static const struct {
        const char* posixMode;
        uint8_t fatfsFlags;
    } modeLut[] = {
        {"r",    FA_READ},
        {"r+",   FA_READ | FA_WRITE},
        {"w",    FA_CREATE_ALWAYS | FA_WRITE},
        {"w+",   FA_CREATE_ALWAYS | FA_WRITE | FA_READ},
        {"a",    FA_OPEN_APPEND | FA_WRITE},
        {"a+",   FA_OPEN_APPEND | FA_WRITE | FA_READ},
        {"wx",   FA_CREATE_NEW | FA_WRITE},
        {"w+x",  FA_CREATE_NEW | FA_WRITE | FA_READ},
        {nullptr, FA_READ}  // Default fallback
    };
    
    if (!mode) return FA_READ;
    
    for (int i = 0; modeLut[i].posixMode; ++i) {
        if (std::strcmp(mode, modeLut[i].posixMode) == 0) {
            return modeLut[i].fatfsFlags;
        }
    }
    
    return FA_READ;  // Default fallback for unknown modes
}

ZP_Error SDFileSystem::init() {
    if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_15) == GPIO_PIN_SET) {
        mounted = false;
        return ZP_ERROR_RESOURCE_UNAVAILABLE;
    }
    // TODO: Ari's version had a HAL_DELAY for 1 second here, that might be needed?
    HAL_Delay(1000); // Wait for SD card to be ready after power up

    int retryCount = 3;
    FRESULT res = f_mount(&fsObj, "", 1);
    while (res != FR_OK && retryCount > 0) {
        HAL_Delay(500); // Wait before retrying
        res = f_mount(&fsObj, "", 1);
        retryCount--;
    }
    mounted = (res == FR_OK);
    return fresultToZpError(res);
}

ZP_Error SDFileSystem::open(File* fp, const char* path, const char* mode) {
    if (!fp) return ZP_ERROR_NULLPTR;
    if (!mounted) return ZP_ERROR_NOT_READY;
    
    FIL* fil = new (&fp->storage[0]) FIL; // Placement new to construct FIL in File's storage
    uint8_t fatfsMode = modeStringToFatfsFlags(mode);
    FRESULT res = f_open(fil, path, fatfsMode);
    return fresultToZpError(res);
}

ZP_Error SDFileSystem::mkdir(const char* path) {
    if (!path) return ZP_ERROR_NULLPTR;
    if (!mounted) return ZP_ERROR_NOT_READY;
    FRESULT res = f_mkdir(path);
    return fresultToZpError(res);
}

ZP_Error SDFileSystem::write(ManagerId_e id, File* fp, const void* buff, uint32_t bytesToWrite, uint32_t* bytesWritten, ReqOptions_e options) {
    if (!fp || !buff) return ZP_ERROR_NULLPTR;
    if (!mounted) return ZP_ERROR_NOT_READY;
    
#ifdef SWO_LOGGING
    swoWrite((const char*)buff, bytesToWrite);
#endif

    if (options == ReqOptions_e::SYNC) {
        uint32_t dummyBytesWritten = 0;
        if (bytesWritten == nullptr) {
            bytesWritten = &dummyBytesWritten; // Use a dummy variable if caller doesn't care about bytes written
        }
        FRESULT res = f_write(reinterpret_cast<FIL*>(&fp->storage[0]), buff, bytesToWrite, reinterpret_cast<UINT*>(bytesWritten));
        res = (res == FR_OK) ? f_sync(reinterpret_cast<FIL*>(&fp->storage[0])) : res; // Sync only if write was successful
        return fresultToZpError(res);
    } else {
        SdReqMsg req;
        req.id = id;
        req.type = ReqType_e::WRITE;
        req.fp = fp;
        req.totalSize = bytesToWrite;
        req.sendResp = (options != ReqOptions_e::ASYNC_NO_RESP);

        SdReqBuf writeBuffMsg;
        while (bytesToWrite > 0) {
            writeBuffMsg.id = id;
            writeBuffMsg.type = ReqType_e::WRITE;
            uint32_t chunkSize = (bytesToWrite < MAX_RW_BUFFER_SIZE) ? bytesToWrite : MAX_RW_BUFFER_SIZE;
            std::memcpy(writeBuffMsg.buff, buff, chunkSize);
            writeBuffMsg.size = chunkSize;
            (chunkSize < MAX_RW_BUFFER_SIZE) ? writeBuffMsg.buff[chunkSize] = '\0' : writeBuffMsg.buff[MAX_RW_BUFFER_SIZE - 1] = '\0'; // Null terminate for safety

            if (bufferQueue->push(&writeBuffMsg) != ZP_ERROR_OK) {
                return ZP_ERROR_FAIL | ZP_ERROR_EXT_API; // Failed to send data
            }
        
            buff = static_cast<const char*>(buff) + chunkSize; // Move buffer pointer forward
            bytesToWrite -= chunkSize; // Decrease remaining byte count
        }

        if (requestQueue->push(&req) != ZP_ERROR_OK) {
            return ZP_ERROR_FAIL | ZP_ERROR_EXT_API; // Failed to send request
        }
        
        return ZP_ERROR_OK; // Request sent
    }
}

ZP_Error SDFileSystem::writeAndSync(ManagerId_e id, File* fp, const void* buff, uint32_t bytesToWrite, ReqOptions_e options) {
    if (!fp || !buff) return ZP_ERROR_NULLPTR;
	if (options == ReqOptions_e::SYNC) return ZP_ERROR_INVALID_ARG;
	if (!mounted) return ZP_ERROR_NOT_READY;

#ifdef SWO_LOGGING
    swoWrite((const char*)buff, bytesToWrite);
#endif

    SdReqMsg req;
    req.id = id;
    req.type = ReqType_e::WRITE_SYNC;
    req.fp = fp;
    req.totalSize = bytesToWrite;
    req.sendResp = (options != ReqOptions_e::ASYNC_NO_RESP);

    SdReqBuf writeBuffMsg;
    while (bytesToWrite > 0) {
        writeBuffMsg.id = id;
        writeBuffMsg.type = ReqType_e::WRITE_SYNC;
        uint32_t chunkSize = (bytesToWrite < MAX_RW_BUFFER_SIZE) ? bytesToWrite : MAX_RW_BUFFER_SIZE;
        std::memcpy(writeBuffMsg.buff, buff, chunkSize);
        writeBuffMsg.size = chunkSize;
        (chunkSize < MAX_RW_BUFFER_SIZE) ? writeBuffMsg.buff[chunkSize] = '\0' : writeBuffMsg.buff[MAX_RW_BUFFER_SIZE - 1] = '\0'; // Null terminate for safety

        if (bufferQueue->push(&writeBuffMsg) != ZP_ERROR_OK) {
            return ZP_ERROR_FAIL | ZP_ERROR_EXT_API; // Failed to send data
        }
        
        buff = static_cast<const char*>(buff) + chunkSize; // Move buffer pointer forward
        bytesToWrite -= chunkSize; // Decrease remaining byte count
    }

    if (requestQueue->push(&req) != ZP_ERROR_OK) {
        return ZP_ERROR_FAIL | ZP_ERROR_EXT_API; // Failed to send request
    }
    
    return ZP_ERROR_OK; // Request sent, waiting for response
}

ZP_Error SDFileSystem::sync(ManagerId_e id, File* fp, ReqOptions_e options) {
    if (!fp) return ZP_ERROR_NULLPTR;
	if (!mounted) return ZP_ERROR_NOT_READY;
    
    if (options == ReqOptions_e::SYNC) {
        FRESULT res = f_sync(reinterpret_cast<FIL*>(&fp->storage[0]));
        return fresultToZpError(res);
    } else {
        SdReqMsg req;
        req.id = id;
        req.type = ReqType_e::SYNC;
        req.fp = fp;
        req.sendResp = (options != ReqOptions_e::ASYNC_NO_RESP);
        
        if (requestQueue->push(&req) != ZP_ERROR_OK) {
            return ZP_ERROR_FAIL | ZP_ERROR_EXT_API; // Failed to send request
        }
        return ZP_ERROR_OK; // Request sent, waiting for response
    }
}

ZP_Error SDFileSystem::stat(const char* path, FileInfo_t* fno) {
    if (!fno) return ZP_ERROR_NULLPTR;
	if (!mounted) return ZP_ERROR_NOT_READY;
    
    FILINFO fatfs_fno;
    FRESULT res = f_stat(path, &fatfs_fno);
    
    if (res == FR_OK) {
        fno->size = fatfs_fno.fsize;
        fno->date = fatfs_fno.fdate;
        fno->time = fatfs_fno.ftime;
        fno->isDir = (fatfs_fno.fattrib & AM_DIR) ? 1 : 0;
        std::strncpy(fno->name, fatfs_fno.fname, 255);
        fno->name[255] = '\0';
    }
    
    return fresultToZpError(res);
}

bool SDFileSystem::available() {
    return mounted;
}

/* TODO: Verify in later PR
ZP_Error SDFileSystem::seek_and_write(ManagerId_e id, File* fp, const void* buff, uint32_t bytesToWrite, uint64_t ofs, ReqOptions_e options) {
	if (options == ReqOptions_e::SYNC) return ZP_ERROR_INVALID_ARG;
    if (!fp || !buff) return ZP_ERROR_NULLPTR;

    #ifdef SWO_LOGGING
        swoWrite((const char*)buff, bytesToWrite);
    #endif

    if (!mounted) return ZP_ERROR_NOT_READY;

    SdReqMsg req;
    req.id = id;
    req.type = ReqType_e::WRITE_SEEK;
    req.fp = fp;
    req.totalSize = bytesToWrite;
    req.offset = ofs;
    req.sendResp = (options != ReqOptions_e::ASYNC_NO_RESP);

    SdReqBuf writeBuffMsg;
    while (bytesToWrite > 0) {
        writeBuffMsg.id = id;
        writeBuffMsg.type = ReqType_e::WRITE_SEEK;
        uint32_t chunkSize = (bytesToWrite < MAX_RW_BUFFER_SIZE) ? bytesToWrite : MAX_RW_BUFFER_SIZE;
        std::memcpy(writeBuffMsg.buff, buff, chunkSize);
        writeBuffMsg.size = chunkSize;
        (chunkSize < MAX_RW_BUFFER_SIZE) ? writeBuffMsg.buff[chunkSize] = '\0' : writeBuffMsg.buff[MAX_RW_BUFFER_SIZE - 1] = '\0'; // Null terminate for safety

        if (bufferQueue->push(&writeBuffMsg) != ZP_ERROR_OK) {
            return ZP_ERROR_FAIL | ZP_ERROR_EXT_API; // Failed to send data
        }
        
        buff = static_cast<const char*>(buff) + chunkSize; // Move buffer pointer forward
        bytesToWrite -= chunkSize; // Decrease remaining byte count
    }

    if (requestQueue->push(&req) != ZP_ERROR_OK) {
        return ZP_ERROR_FAIL | ZP_ERROR_EXT_API; // Failed to send request
    }
    
    return ZP_ERROR_OK; // Request sent, waiting for response
}


ZP_Error SDFileSystem::close(File* fp) {
    if (!fp) return ZP_ERROR_NULLPTR;
    
    FIL* fil = reinterpret_cast<FIL*>(&fp->storage[0]);
    FRESULT res = f_close(fil);
    return fresultToZpError(res);
}

ZP_Error SDFileSystem::read(File* fp, void* buff, uint32_t btr, uint32_t* br) {
    if (!fp || !buff) return ZP_ERROR_NULLPTR;
    
    FRESULT res = f_read(reinterpret_cast<FIL*>(&fp->storage[0]), buff, btr, reinterpret_cast<UINT*>(br));
    return fresultToZpError(res);
}

ZP_Error SDFileSystem::lseek(ManagerId_e id, File* fp, uint64_t ofs, ReqOptions_e options) {
    if (!fp) return ZP_ERROR_NULLPTR;
    
    if (options == ReqOptions_e::SYNC) {
        FRESULT res = f_lseek(reinterpret_cast<FIL*>(&fp->storage[0]), static_cast<FSIZE_t>(ofs));
        return fresultToZpError(res);
    } else {
        SdReqMsg req;
        req.id = id;
        req.type = ReqType_e::LSEEK;
        req.fp = fp;
        req.offset = ofs;

        if (requestQueue->push(&req) != ZP_ERROR_OK) {
            return ZP_ERROR_FAIL | ZP_ERROR_EXT_API; // Failed to send request
        }
        return ZP_ERROR_OK; // Request sent, waiting for response
    }
}

ZP_Error SDFileSystem::tell(ManagerId_e id, File* fp, uint64_t* position, ReqOptions_e options) {
    if (!fp) return ZP_ERROR_NULLPTR; // we dont need position because this operation is async for SD
    
    if (options == ReqOptions_e::SYNC) {
        DWORD pos = f_tell(reinterpret_cast<FIL*>(&fp->storage[0]));
        if (pos == 0xFFFFFFFF) { // f_tell returns 0xFFFFFFFF on error
            return ZP_ERROR_FAIL | ZP_ERROR_EXT_API;
        }
        if (position) {
            *position = static_cast<uint64_t>(pos);
        }
        return ZP_ERROR_OK;
    } else {
        if (options == ReqOptions_e::ASYNC_NO_RESP) {
            return ZP_ERROR_INVALID_ARG; // TELL operation requires a response to return the position, so ASYNC_NO_RESP is not valid here
        }
        SdReqMsg req;
        req.id = id;
        req.type = ReqType_e::TELL;
        req.fp = fp;

        if (requestQueue->push(&req) != ZP_ERROR_OK) {
            return ZP_ERROR_FAIL | ZP_ERROR_EXT_API; // Failed to send request
        }
        return ZP_ERROR_OK; // Request sent, waiting for response
    }
}

int SDFileSystem::printf(ManagerId_e id, File* fp, ReqOptions_e options, const char* str, ...) {
    char printBuff[MAX_RW_BUFFER_SIZE];
    va_list args;
    va_start(args, str);
    int len = std::vsnprintf(printBuff, MAX_RW_BUFFER_SIZE, str, args);
    va_end(args);

    if (len < 0) {
        return EOF; // Encoding error
    }
#ifdef SWO_LOGGING
    swoWrite(printBuff, static_cast<uint32_t>(len));
#endif

    if (!fp || !mounted) return EOF;
    
    if (options == ReqOptions_e::SYNC) {
        FIL* fil = reinterpret_cast<FIL*>(&fp->storage[0]);
        va_list args;
        va_start(args, str);
        int res = f_printf(fil, str, args);
        va_end(args);
        return res;
    } else {
        uint32_t bytesToWrite = (len < MAX_RW_BUFFER_SIZE) ? static_cast<uint32_t>(len) : (MAX_RW_BUFFER_SIZE - 1);
        printBuff[bytesToWrite] = '\0'; // Ensure null termination

        return write(id, fp, printBuff, bytesToWrite, nullptr, options) == ZP_ERROR_OK ? bytesToWrite : EOF;
    }
}

PollResult SDFileSystem::poll(ManagerId_e id, ReqType_e reqType) {
    PollResult result;
    result.type = reqType;
    result.status |= ZP_ERROR_NOT_READY; // Default to not ready

    if (responseQueues[static_cast<size_t>(id)]->get(&result) == ZP_ERROR_OK) {
        return result;
    } else {
        result.status |= ZP_ERROR_BUSY;
        return result;
    }
}
*/

