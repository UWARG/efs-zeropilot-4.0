#include "sd.hpp"
#include <new>
#include <cstring>
#include <cstdio>
#include <cstdarg>

SDFileSystem::SDFileSystem(MessageQueue<FatFSReqMsg> *reqQueue, MessageQueue<FatFSReqBuff> *buffQueue, MessageQueue<PollResult> *respQueues[static_cast<size_t>(ManId::COUNT)]) 
    : mounted(false), requestQueue(reqQueue), bufferQueue(buffQueue), responseQueues(respQueues) {
    std::memset(&fsObj, 0, sizeof(FATFS));
}

SDFileSystem::~SDFileSystem() {
    if (mounted) {
        f_mount(nullptr, "", 0);
    }
}

FileStatus SDFileSystem::fresultToStatus(FRESULT res) {
    return (res == FR_OK) ? FILE_STATUS_OK : FILE_STATUS_ERROR;
}

BYTE SDFileSystem::modeStringToFatfsFlags(const char* mode) {
    static const struct {
        const char* posix_mode;
        BYTE fatfs_flags;
    } mode_lut[] = {
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
    
    for (int i = 0; mode_lut[i].posix_mode; ++i) {
        if (std::strcmp(mode, mode_lut[i].posix_mode) == 0) {
            return mode_lut[i].fatfs_flags;
        }
    }
    
    return FA_READ;  // Default fallback for unknown modes
}

FileStatus SDFileSystem::open(File* fp, const char* path, const char* mode) {
    if (!fp) return FILE_STATUS_ERROR;
    
    FIL* fil = new (&fp->_storage[0]) FIL; // Placement new to construct FIL in File's storage
    BYTE fatfs_mode = modeStringToFatfsFlags(mode);
    FRESULT res = f_open(fil, path, fatfs_mode);
    return fresultToStatus(res);
}

FileStatus SDFileSystem::close(File* fp) {
    if (!fp) return FILE_STATUS_ERROR;
    
    FIL* fil = reinterpret_cast<FIL*>(&fp->_storage[0]);
    FRESULT res = f_close(fil);
    return fresultToStatus(res);
}

FileStatus SDFileSystem::read(File* fp, void* buff, uint32_t btr, uint32_t* br) {
    if (!fp || !buff) return FILE_STATUS_ERROR;
    
    FRESULT res = f_read(reinterpret_cast<FIL*>(&fp->_storage[0]), buff, btr, reinterpret_cast<UINT*>(br));
    return fresultToStatus(res);
}

FileStatus SDFileSystem::write(ManId id, File* fp, const void* buff, uint32_t btw, uint32_t* bw, ReqOptions options) {
    if (!fp || !buff) return FILE_STATUS_ERROR;
    
#ifdef SWO_LOGGING
    ::printf("%.*s", btw, (const char*)buff);
#endif

    if (!mounted) return FILE_STATUS_ERROR;

    if (options == ReqOptions::SYNC) {
        uint32_t dummy_bw = 0;
        if (bw == nullptr) {
            bw = &dummy_bw; // Use a dummy variable if caller doesn't care about bytes written
        }
        FRESULT res = f_write(reinterpret_cast<FIL*>(&fp->_storage[0]), buff, btw, reinterpret_cast<UINT*>(bw));
        res = (res == FR_OK) ? f_sync(reinterpret_cast<FIL*>(&fp->_storage[0])) : res; // Sync only if write was successful
        return fresultToStatus(res);
    } else {
        FatFSReqMsg req;
        req.id = id;
        req.type = ReqType::WRITE;
        req.fp = reinterpret_cast<FIL*>(&fp->_storage[0]);
        req.total_size = btw;
        req.sendResp = (options != ReqOptions::ASYNC_NO_RESP);

        FatFSReqBuff writeBuffMsg;
        while (btw > 0) {
            writeBuffMsg.id = id;
            writeBuffMsg.type = ReqType::WRITE;
            uint32_t chunkSize = (btw < MAX_RW_BUFFER_SIZE) ? btw : MAX_RW_BUFFER_SIZE;
            std::memcpy(writeBuffMsg.buff, buff, chunkSize);
            writeBuffMsg.size = chunkSize;
            (chunkSize < MAX_RW_BUFFER_SIZE) ? writeBuffMsg.buff[chunkSize] = '\0' : writeBuffMsg.buff[MAX_RW_BUFFER_SIZE - 1] = '\0'; // Null terminate for safety

            if (bufferQueue->push(&writeBuffMsg) != osOK) {
                return FILE_STATUS_ERROR; // Failed to send data
            }
        
            buff = static_cast<const char*>(buff) + chunkSize; // Move buffer pointer forward
            btw -= chunkSize; // Decrease remaining byte count
        }

        if (requestQueue->push(&req) != osOK) {
            return FILE_STATUS_ERROR; // Failed to send request
        }
        
        return FILE_STATUS_REQUEST_MADE; // Request sent
    }
}

FileStatus SDFileSystem::seek_and_write(ManId id, File* fp, const void* buff, uint32_t btw, uint64_t ofs, ReqOptions options) {
    if (!fp || !buff || options == ReqOptions::SYNC) return FILE_STATUS_ERROR;

#ifdef SWO_LOGGING
    ::printf("%.*s", ofs, btw, (const char*)buff);
#endif

    if (!mounted) return FILE_STATUS_ERROR;
    
    FatFSReqMsg req;
    req.id = id;
    req.type = ReqType::WRITE_SEEK;
    req.fp = reinterpret_cast<FIL*>(&fp->_storage[0]);
    req.total_size = btw;
    req.offset = ofs;
    req.sendResp = (options != ReqOptions::ASYNC_NO_RESP);

    FatFSReqBuff writeBuffMsg;
    while (btw > 0) {
        writeBuffMsg.id = id;
        writeBuffMsg.type = ReqType::WRITE_SEEK;
        uint32_t chunkSize = (btw < MAX_RW_BUFFER_SIZE) ? btw : MAX_RW_BUFFER_SIZE;
        std::memcpy(writeBuffMsg.buff, buff, chunkSize);
        writeBuffMsg.size = chunkSize;
        (chunkSize < MAX_RW_BUFFER_SIZE) ? writeBuffMsg.buff[chunkSize] = '\0' : writeBuffMsg.buff[MAX_RW_BUFFER_SIZE - 1] = '\0'; // Null terminate for safety

        if (bufferQueue->push(&writeBuffMsg) != osOK) {
            return FILE_STATUS_ERROR; // Failed to send data
        }
        
        buff = static_cast<const char*>(buff) + chunkSize; // Move buffer pointer forward
        btw -= chunkSize; // Decrease remaining byte count
    }

    if (requestQueue->push(&req) != osOK) {
        return FILE_STATUS_ERROR; // Failed to send request
    }
    
    return FILE_STATUS_REQUEST_MADE; // Request sent, waiting for response
}

FileStatus SDFileSystem::write_and_sync(ManId id, File* fp, const void* buff, uint32_t btw, ReqOptions options) {
    if (!fp || !buff || options == ReqOptions::SYNC) return FILE_STATUS_ERROR;

#ifdef SWO_LOGGING
    ::printf("%.*s", btw, (const char*)buff);
#endif

    if (!mounted) return FILE_STATUS_ERROR;
    
    FatFSReqMsg req;
    req.id = id;
    req.type = ReqType::WRITE_SYNC;
    req.fp = reinterpret_cast<FIL*>(&fp->_storage[0]);
    req.total_size = btw;
    req.sendResp = (options != ReqOptions::ASYNC_NO_RESP);

    FatFSReqBuff writeBuffMsg;
    while (btw > 0) {
        writeBuffMsg.id = id;
        writeBuffMsg.type = ReqType::WRITE_SYNC;
        uint32_t chunkSize = (btw < MAX_RW_BUFFER_SIZE) ? btw : MAX_RW_BUFFER_SIZE;
        std::memcpy(writeBuffMsg.buff, buff, chunkSize);
        writeBuffMsg.size = chunkSize;
        (chunkSize < MAX_RW_BUFFER_SIZE) ? writeBuffMsg.buff[chunkSize] = '\0' : writeBuffMsg.buff[MAX_RW_BUFFER_SIZE - 1] = '\0'; // Null terminate for safety

        if (bufferQueue->push(&writeBuffMsg) != osOK) {
            return FILE_STATUS_ERROR; // Failed to send data
        }
        
        buff = static_cast<const char*>(buff) + chunkSize; // Move buffer pointer forward
        btw -= chunkSize; // Decrease remaining byte count
    }

    if (requestQueue->push(&req) != osOK) {
        return FILE_STATUS_ERROR; // Failed to send request
    }
    
    return FILE_STATUS_REQUEST_MADE; // Request sent, waiting for response
}

FileStatus SDFileSystem::lseek(ManId id, File* fp, uint64_t ofs, ReqOptions options) {
    if (!fp) return FILE_STATUS_ERROR;
    
    if (options == ReqOptions::SYNC) {
        FRESULT res = f_lseek(reinterpret_cast<FIL*>(&fp->_storage[0]), static_cast<FSIZE_t>(ofs));
        return fresultToStatus(res);
    } else {
        FatFSReqMsg req;
        req.id = id;
        req.type = ReqType::LSEEK;
        req.fp = reinterpret_cast<FIL*>(&fp->_storage[0]);
        req.offset = ofs;

        if (requestQueue->push(&req) != osOK) {
            return FILE_STATUS_ERROR; // Failed to send request
        }
        return FILE_STATUS_REQUEST_MADE; // Request sent, waiting for response
    }
}

FileStatus SDFileSystem::tell(ManId id, File* fp, uint64_t* position, ReqOptions options) {
    if (!fp) return FILE_STATUS_ERROR; // we dont need position because this operation is async for SD
    
    if (options == ReqOptions::SYNC) {
        DWORD pos = f_tell(reinterpret_cast<FIL*>(&fp->_storage[0]));
        if (pos == 0xFFFFFFFF) { // f_tell returns 0xFFFFFFFF on error
            return FILE_STATUS_ERROR;
        }
        if (position) {
            *position = static_cast<uint64_t>(pos);
        }
        return FILE_STATUS_OK;
    } else {
        if (options == ReqOptions::ASYNC_NO_RESP) {
            return FILE_STATUS_ERROR; // TELL operation requires a response to return the position, so ASYNC_NO_RESP is not valid here
        }
        FatFSReqMsg req;
        req.id = id;
        req.type = ReqType::TELL;
        req.fp = reinterpret_cast<FIL*>(&fp->_storage[0]);

        if (requestQueue->push(&req) != osOK) {
            return FILE_STATUS_ERROR; // Failed to send request
        }
        return FILE_STATUS_REQUEST_MADE; // Request sent, waiting for response
    }
}

FileStatus SDFileSystem::sync(ManId id, File* fp, ReqOptions options) {
    if (!fp) return FILE_STATUS_ERROR;

    if (options == ReqOptions::SYNC) {
        FRESULT res = f_sync(reinterpret_cast<FIL*>(&fp->_storage[0]));
        return fresultToStatus(res);
    } else {
        FatFSReqMsg req;
        req.id = id;
        req.type = ReqType::SYNC;
        req.fp = reinterpret_cast<FIL*>(&fp->_storage[0]);
        req.sendResp = (options != ReqOptions::ASYNC_NO_RESP);

        if (requestQueue->push(&req) != osOK) {
            return FILE_STATUS_ERROR; // Failed to send request
        }
        return FILE_STATUS_REQUEST_MADE; // Request sent, waiting for response
    }
}

FileStatus SDFileSystem::mkdir(const char* path) {
    if (!path) return FILE_STATUS_ERROR;
    FRESULT res = f_mkdir(path);
    return fresultToStatus(res);
}

FileStatus SDFileSystem::stat(const char* path, FileInfo* fno) {
    if (!fno) return FILE_STATUS_ERROR;
    
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
    
    return fresultToStatus(res);
}

int SDFileSystem::printf(ManId id, File* fp, ReqOptions options, const char* str, ...) {
    char printBuff[MAX_RW_BUFFER_SIZE];
    va_list args;
    va_start(args, str);
    int len = std::vsnprintf(printBuff, MAX_RW_BUFFER_SIZE, str, args);
    va_end(args);

    if (len < 0) {
        return EOF; // Encoding error
    }
#ifdef SWO_LOGGING
    ::printf("%.*s", len, printBuff);
#endif

    if (!fp || !mounted) return EOF;
    
    if (options == ReqOptions::SYNC) {
        FIL* fil = reinterpret_cast<FIL*>(&fp->_storage[0]);
        va_list args;
        va_start(args, str);
        int res = f_printf(fil, str, args);
        va_end(args);
        return res;
    } else {
        uint32_t bytesToWrite = (len < MAX_RW_BUFFER_SIZE) ? static_cast<uint32_t>(len) : (MAX_RW_BUFFER_SIZE - 1);
        printBuff[bytesToWrite] = '\0'; // Ensure null termination

        return write(id, fp, printBuff, bytesToWrite, nullptr, options) == FILE_STATUS_REQUEST_MADE ? bytesToWrite : EOF;
    }
}

FileStatus SDFileSystem::init() {
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
    return fresultToStatus(res);
}

PollResult SDFileSystem::poll(ManId id, ReqType reqType) {
    PollResult result;
    result.type = reqType;
    result.status = FILE_STATUS_NOT_DONE; // Default to not done

    if (responseQueues[static_cast<size_t>(id)]->get(&result) == osOK) {
        return result;
    } else {
        result.status = FILE_STATUS_ERROR;
        return result;
    }
}

bool SDFileSystem::available() {
    return mounted;
}
