#include "sd.hpp"
#include <cstring>
#include <cstdio>
#include <cstdarg>

SDFileSystem::SDFileSystem(MessageQueue<FatFSReqMsg> *reqQueue, MessageQueue<FatFSReqBuff> *buffQueue, MessageQueue<FatFSRespMsg> *respQueues[ManId::COUNT]) 
    : requestQueue(reqQueue), bufferQueue(buffQueue), responseQueues(respQueues), mounted(false) {
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
    
    FIL* fil = reinterpret_cast<FIL*>(fp->_storage);
    BYTE fatfs_mode = modeStringToFatfsFlags(mode);
    FRESULT res = f_open(fil, path, fatfs_mode);
    return fresultToStatus(res);
}

FileStatus SDFileSystem::close(File* fp, bool forceSync) {
    if (!fp) return FILE_STATUS_ERROR;
    
    FIL* fil = reinterpret_cast<FIL*>(fp->_storage);
    FRESULT res = f_close(fil);
    return fresultToStatus(res);
}

FileStatus SDFileSystem::read(File* fp, void* buff, uint32_t btr, uint32_t* br) {
    if (!fp || !buff) return FILE_STATUS_ERROR;
    
    FRESULT res = f_read(reinterpret_cast<FIL*>(fp->_storage), buff, btr, reinterpret_cast<UINT*>(br));
    return fresultToStatus(res);
}

FileStatus SDFileSystem::write(File* fp, const void* buff, uint32_t btw, uint32_t* bw, ReqOptions options) {
    if (!fp || !buff) return FILE_STATUS_ERROR;
    
#ifdef SWO_LOGGING
    if (res == FR_OK && bw && *bw > 0) {
        ::printf("%.*s", *bw, (const char*)buff);
    }
#endif

    if (options == ReqOptions::SYNC) {
        if (bw == nullptr) {
            uint32_t dummy_bw;
            bw = &dummy_bw; // Use a dummy variable if caller doesn't care about bytes written
        }
        FRESULT res = f_write(reinterpret_cast<FIL*>(fp->_storage), buff, btw, reinterpret_cast<UINT*>(bw));
        return fresultToStatus(res);
    } else {
        FatFsReqMsg req;
        req.type = ReqType::WRITE;
        req.fp = reinterpret_cast<FIL*>(fp->_storage);
        req.total_size = btw;
        req.sendResp = (options != ReqOptions::ASYNC_NO_RESP);

        FatFSReqBuff writeBuffMsg;
        std::memcpy(writeBuffMsg.buff, buff, (btw < MAX_RW_BUFFER_SIZE) ? btw : MAX_RW_BUFFER_SIZE);
        writeBuffMsg.size = (btw < MAX_RW_BUFFER_SIZE) ? btw : MAX_RW_BUFFER_SIZE;
        (btw < MAX_RW_BUFFER_SIZE) ? writeBuffMsg.buff[btw] = '\0' : writeBuffMsg.buff[MAX_RW_BUFFER_SIZE - 1] = '\0'; // Null terminate for safety

        if (requestQueue->put(&req) != osOK || bufferQueue->put(&writeBuffMsg) != osOK) {
            return FILE_STATUS_ERROR; // Failed to send request
        }
        
        return FILE_STATUS_REQUEST_MADE; // Request sent
    }
}

FileStatus SDFileSystem::seek_and_write(File* fp, const void* buff, uint32_t btw, uint32_t* bw, uint64_t ofs) {
    if (!fp || !buff) return FILE_STATUS_ERROR;
    
    FatFsReqMsg req;
    req.type = ReqType::WRITE_SEEK;
    req.fp = reinterpret_cast<FIL*>(fp->_storage);
    req.total_size = btw;
    req.offset = ofs;

    FatFSReqBuff writeBuffMsg;
    std::memcpy(writeBuffMsg.buff, buff, (btw < MAX_RW_BUFFER_SIZE) ? btw : MAX_RW_BUFFER_SIZE);
    writeBuffMsg.size = (btw < MAX_RW_BUFFER_SIZE) ? btw : MAX_RW_BUFFER_SIZE;
    (btw < MAX_RW_BUFFER_SIZE) ? writeBuffMsg.buff[btw] = '\0' : writeBuffMsg.buff[MAX_RW_BUFFER_SIZE - 1] = '\0'; // Null terminate for safety

    if (requestQueue->put(&req) != osOK || bufferQueue->put(&writeBuffMsg) != osOK) {
        return FILE_STATUS_ERROR; // Failed to send request
    }
    
    return FILE_STATUS_REQUEST_MADE; // Request sent, waiting for response
}

FileStatus SDFileSystem::write_and_sync(File* fp, const void* buff, uint32_t btw, uint32_t* bw) {
    if (!fp || !buff) return FILE_STATUS_ERROR;
    
    FatFsReqMsg req;
    req.type = ReqType::WRITE_SYNC;
    req.fp = reinterpret_cast<FIL*>(fp->_storage);
    req.total_size = btw;

    FatFSReqBuff writeBuffMsg;
    std::memcpy(writeBuffMsg.buff, buff, (btw < MAX_RW_BUFFER_SIZE) ? btw : MAX_RW_BUFFER_SIZE);
    writeBuffMsg.size = (btw < MAX_RW_BUFFER_SIZE) ? btw : MAX_RW_BUFFER_SIZE;
    (btw < MAX_RW_BUFFER_SIZE) ? writeBuffMsg.buff[btw] = '\0' : writeBuffMsg.buff[MAX_RW_BUFFER_SIZE - 1] = '\0'; // Null terminate for safety

    if (requestQueue->put(&req) != osOK || bufferQueue->put(&writeBuffMsg) != osOK) {
        return FILE_STATUS_ERROR; // Failed to send request
    }
    
    return FILE_STATUS_REQUEST_MADE; // Request sent, waiting for response
}

FileStatus SDFileSystem::lseek(File* fp, uint64_t ofs, ReqOptions options) {
    if (!fp) return FILE_STATUS_ERROR;
    
    if (options == ReqOptions::SYNC) {
        FRESULT res = f_lseek(reinterpret_cast<FIL*>(fp->_storage), static_cast<FSIZE_t>(ofs));
        return fresultToStatus(res);
    } else {
        FatFSReqMsg req;
        req.type = ReqType::LSEEK;
        req.fp = reinterpret_cast<FIL*>(fp->_storage);
        req.offset = ofs;

        if (requestQueue->put(&req) != osOK) {
            return FILE_STATUS_ERROR; // Failed to send request
        }
        return FILE_STATUS_REQUEST_MADE; // Request sent, waiting for response
    }
}

FileStatus SDFileSystem::tell(File* fp, uint64_t* position, ReqOptions options) {
    if (!fp) return FILE_STATUS_ERROR; // we dont need position because this operation is async for SD
    
    if (options == ReqOptions::SYNC) {
        DWORD pos = f_tell(reinterpret_cast<FIL*>(fp->_storage));
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
        req.type = ReqType::TELL;
        req.fp = reinterpret_cast<FIL*>(fp->_storage);

        if (requestQueue->put(&req) != osOK) {
            return FILE_STATUS_ERROR; // Failed to send request
        }
        return FILE_STATUS_REQUEST_MADE; // Request sent, waiting for response
    }
}

FileStatus SDFileSystem::sync(File* fp, ReqOptions options) {
    if (!fp) return FILE_STATUS_ERROR;

    if (options == ReqOptions::SYNC) {
        FRESULT res = f_sync(reinterpret_cast<FIL*>(fp->_storage));
        return fresultToStatus(res);
    } else {
        FatFsReqMsg req;
        req.type = ReqType::SYNC;
        req.fp = reinterpret_cast<FIL*>(fp->_storage);
        req.sendResp = (options != ReqOptions::ASYNC_NO_RESP);

        if (requestQueue->put(&req) != osOK) {
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
    if (!fno && forceSync) return FILE_STATUS_ERROR; // fno is needed to return stat info when forceSync is true
    
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

int SDFileSystem::printf(File* fp, const char* str, ...) {
    if (!fp) return EOF;
    
    FIL* fil = reinterpret_cast<FIL*>(fp->_storage);
    va_list args;
    va_start(args, str);
    int res = f_printf(fil, str, args);
    va_end(args);
    return res;
}

FileStatus SDFileSystem::init() {
    // TODO: Ari's version had a HAL_DELAY for 1 second here, that might be needed?

    FRESULT res = f_mount(&fsObj, "", 1);
    mounted = (res == FR_OK);
    return fresultToStatus(res);
}

bool SDFileSystem::available() {
    return mounted;
}
