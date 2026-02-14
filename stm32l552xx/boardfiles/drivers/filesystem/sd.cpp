#include "sd.hpp"
#include <cstring>
#include <cstdio>
#include <cstdarg>

SDFileSystem::SDFileSystem() : mounted(false) {
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
    BYTE flags = modeStringToFatfsFlags(mode);
    
    FRESULT res = f_open(fil, path, flags);
    return fresultToStatus(res);
}

FileStatus SDFileSystem::close(File* fp) {
    if (!fp) return FILE_STATUS_ERROR;
    
    FIL* fil = reinterpret_cast<FIL*>(fp->_storage);
    FRESULT res = f_close(fil);
    return fresultToStatus(res);
}

FileStatus SDFileSystem::read(File* fp, void* buff, uint32_t btr, uint32_t* br) {
    if (!fp || !buff) return FILE_STATUS_ERROR;
    
    FIL* fil = reinterpret_cast<FIL*>(fp->_storage);
    FRESULT res = f_read(fil, buff, btr, br);
    return fresultToStatus(res);
}

FileStatus SDFileSystem::write(File* fp, const void* buff, uint32_t btw, uint32_t* bw) {
    if (!fp || !buff) return FILE_STATUS_ERROR;
    
    FIL* fil = reinterpret_cast<FIL*>(fp->_storage);
    FRESULT res = f_write(fil, buff, btw, bw);
    
#ifdef SWO_LOGGING
    if (res == FR_OK && bw && *bw > 0) {
        ::printf("%.*s", *bw, (const char*)buff);
    }
#endif
    
    return fresultToStatus(res);
}

FileStatus SDFileSystem::lseek(File* fp, uint64_t ofs) {
    if (!fp) return FILE_STATUS_ERROR;
    
    FIL* fil = reinterpret_cast<FIL*>(fp->_storage);
    FRESULT res = f_lseek(fil, ofs);
    return fresultToStatus(res);
}

FileStatus SDFileSystem::truncate(File* fp) {
    if (!fp) return FILE_STATUS_ERROR;
    
    FIL* fil = reinterpret_cast<FIL*>(fp->_storage);
    FRESULT res = f_truncate(fil);
    return fresultToStatus(res);
}

FileStatus SDFileSystem::rewind(File* fp) {
    if (!fp) return FILE_STATUS_ERROR;
    
    FIL* fil = reinterpret_cast<FIL*>(fp->_storage);
    FRESULT res = f_lseek(fil, 0);
    return fresultToStatus(res);
}

FileStatus SDFileSystem::tell(File* fp, uint64_t* position) {
    if (!fp || !position) return FILE_STATUS_ERROR;
    
    FIL* fil = reinterpret_cast<FIL*>(fp->_storage);
    *position = f_tell(fil);
    return FILE_STATUS_OK;
}

FileStatus SDFileSystem::sync(File* fp) {
    if (!fp) return FILE_STATUS_ERROR;
    
    FIL* fil = reinterpret_cast<FIL*>(fp->_storage);
    FRESULT res = f_sync(fil);
    return fresultToStatus(res);
}

FileStatus SDFileSystem::mkdir(const char* path) {
    FRESULT res = f_mkdir(path);
    return fresultToStatus(res);
}

FileStatus SDFileSystem::unlink(const char* path) {
    FRESULT res = f_unlink(path);
    return fresultToStatus(res);
}

FileStatus SDFileSystem::rename(const char* path_old, const char* path_new) {
    FRESULT res = f_rename(path_old, path_new);
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

int SDFileSystem::putc(char c, File* fp) {
    if (!fp) return EOF;
    
    FIL* fil = reinterpret_cast<FIL*>(fp->_storage);
    return f_putc(c, fil);
}

int SDFileSystem::puts(const char* str, File* cp) {
    if (!cp) return EOF;
    
    FIL* fil = reinterpret_cast<FIL*>(cp->_storage);
    int res = f_puts(str, fil);
    
#ifdef SWO_LOGGING
    if (res >= 0) {
        ::printf("%s", str);
    }
#endif
    
    return res;
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

char* SDFileSystem::gets(char* buff, int len, File* fp) {
    if (!buff || !fp) return nullptr;
    
    FIL* fil = reinterpret_cast<FIL*>(fp->_storage);
    return f_gets(buff, len, fil);
}

FileStatus SDFileSystem::init() {
    // TODO: Ari's version had a HAL_DELAY for 1 second here, that might be needed?

    FRESULT res = f_mount(&fsObj, "", 1);
    mounted = (res == FR_OK);
    return fresultToStatus(res);
}
