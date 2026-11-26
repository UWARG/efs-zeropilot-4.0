#pragma once
#include "filesystem_iface.hpp"
#include <cstdio>
#include <cstring>
#include <cstdarg>
#include <sys/stat.h>

// Platform-specific
#ifdef _WIN32
    #include <direct.h>
    #define PLATFORM_MKDIR(path) _mkdir(path)
    #define PLATFORM_TRUNCATE(file, pos) _chsize_s(_fileno(file), pos)
#else
    #include <unistd.h>
    #define PLATFORM_MKDIR(path) ::mkdir(path, 0755)
    #define PLATFORM_TRUNCATE(file, pos) ftruncate(fileno(file), pos)
#endif

class SITL_FileSystem : public IFileSystem {
private:
    using FileHandle = ::FILE;
    
    // Helper to get FILE* from File struct
    static FileHandle* getFileHandle(File* fp) {
        if (!fp) return nullptr;
        return *reinterpret_cast<FileHandle**>(&fp->storage[0]);
    }
    
    // Helper to set FILE* in File struct
    static void setFileHandle(File* fp, FileHandle* handle) {
        if (!fp) return;
        *reinterpret_cast<FileHandle**>(&fp->storage[0]) = handle;
    }

public:
    SITL_FileSystem() = default;
    ~SITL_FileSystem() = default;

    ZP_Error open(File* fp, const char* path, const char* mode) override {
        if (!fp || !path || !mode) return ZP_ERROR_NULLPTR;
        
        FileHandle* file = std::fopen(path, mode);
        if (!file) return ZP_ERROR_FAIL | ZP_ERROR_EXT_API;
        
        setFileHandle(fp, file);
        return ZP_ERROR_OK;
    }

    ZP_Error write(ManagerId_e id, File* fp, const void* buff, uint32_t bytesToWrite,
                       uint32_t* bytesWritten, ReqOptions_e options = ReqOptions_e::ASYNC) override {
        (void)id;
        (void)options;
        if (!fp || !buff) return ZP_ERROR_NULLPTR;
        FileHandle* file = getFileHandle(fp);
        if (!file) return ZP_ERROR_FAIL | ZP_ERROR_EXT_API;

        size_t bytes_written = std::fwrite(buff, 1, bytesToWrite, file);
        if (bytesWritten) *bytesWritten = static_cast<uint32_t>(bytes_written);

        if (std::ferror(file)) return ZP_ERROR_FAIL | ZP_ERROR_EXT_API;
        return ZP_ERROR_OK;
    }

    ZP_Error writeAndSync(ManagerId_e id, File* fp, const void* buff, uint32_t bytesToWrite,
                              ReqOptions_e options = ReqOptions_e::ASYNC) override {
        ZP_Error status = write(id, fp, buff, bytesToWrite, nullptr, options);
        if (status != ZP_ERROR_OK) return status;
        return sync(id, fp, options);
    }

    ZP_Error sync(ManagerId_e id, File* fp,
                      ReqOptions_e options = ReqOptions_e::ASYNC) override {
        (void)id;
        (void)options;
        if (!fp) return ZP_ERROR_NULLPTR;
        FileHandle* file = getFileHandle(fp);
        if (!file) return ZP_ERROR_FAIL | ZP_ERROR_EXT_API;

        if (std::fflush(file) != 0) return ZP_ERROR_FAIL | ZP_ERROR_EXT_API;
        return ZP_ERROR_OK;
    }

    ZP_Error mkdir(const char* path) override {
        if (!path) return ZP_ERROR_NULLPTR;
        if (PLATFORM_MKDIR(path) != 0) return ZP_ERROR_FAIL | ZP_ERROR_EXT_API;
        return ZP_ERROR_OK;
    }

    ZP_Error stat(const char* path, FileInfo_t* fno) override {
        if (!path || !fno) return ZP_ERROR_NULLPTR;

        struct stat st;
        if (::stat(path, &st) != 0) return ZP_ERROR_FAIL | ZP_ERROR_EXT_API;

        fno->size = static_cast<uint64_t>(st.st_size);
        fno->isDir = S_ISDIR(st.st_mode) ? 1 : 0;
        std::strncpy(fno->name, path, 255);
        fno->name[255] = '\0';

        return ZP_ERROR_OK;
    }

    bool available() override {
        // The host filesystem is always mounted/usable in SITL.
        return true;
    }

    // ---- SITL-only helpers (not part of IFileSystem) ------------------------

    /* TODO: Verify for later PR
    ZP_Error close(File* fp) override {
        if (!fp) return ZP_ERROR_NULLPTR;
        FileHandle* file = getFileHandle(fp);
        if (!file) return ZP_ERROR_FAIL | ZP_ERROR_EXT_API;
        
        if (std::fclose(file) != 0) return ZP_ERROR_FAIL | ZP_ERROR_EXT_API;
        setFileHandle(fp, nullptr);
        return ZP_ERROR_OK;
    }

    ZP_Error read(File* fp, void* buff, uint32_t btr, uint32_t* br) override {
        if (!fp || !buff) return ZP_ERROR_NULLPTR;
        FileHandle* file = getFileHandle(fp);
        if (!file) return ZP_ERROR_FAIL | ZP_ERROR_EXT_API;
        
        size_t bytes_read = std::fread(buff, 1, btr, file);
        if (br) *br = static_cast<uint32_t>(bytes_read);
        
        if (std::ferror(file)) return ZP_ERROR_FAIL | ZP_ERROR_EXT_API;
        return ZP_ERROR_OK;
    }

    ZP_Error lseek(File* fp, uint64_t ofs) override {
        if (!fp) return ZP_ERROR_NULLPTR;
        FileHandle* file = getFileHandle(fp);
        if (!file) return ZP_ERROR_FAIL | ZP_ERROR_EXT_API;
        
        if (std::fseek(file, static_cast<long>(ofs), SEEK_SET) != 0) {
            return ZP_ERROR_FAIL | ZP_ERROR_EXT_API;
        }
        return ZP_ERROR_OK;
    }

    ZP_Error truncate(File* fp) override {
        if (!fp) return ZP_ERROR_NULLPTR;
        FileHandle* file = getFileHandle(fp);
        if (!file) return ZP_ERROR_FAIL | ZP_ERROR_EXT_API;
        
        long pos = std::ftell(file);
        if (pos < 0) return ZP_ERROR_FAIL | ZP_ERROR_EXT_API;
        
        if (PLATFORM_TRUNCATE(file, pos) != 0) return ZP_ERROR_FAIL | ZP_ERROR_EXT_API;
        return ZP_ERROR_OK;
    }

    ZP_Error rewind(File* fp) override {
        if (!fp) return ZP_ERROR_NULLPTR;
        FileHandle* file = getFileHandle(fp);
        if (!file) return ZP_ERROR_FAIL | ZP_ERROR_EXT_API;
        
        std::rewind(file);
        return ZP_ERROR_OK;
    }

    ZP_Error tell(File* fp, uint64_t* position) override {
        if (!fp || !position) return ZP_ERROR_NULLPTR;
        FileHandle* file = getFileHandle(fp);
        if (!file) return ZP_ERROR_FAIL | ZP_ERROR_EXT_API;
        
        long pos = std::ftell(file);
        if (pos < 0) return ZP_ERROR_FAIL | ZP_ERROR_EXT_API;
        
        *position = static_cast<uint64_t>(pos);
        return ZP_ERROR_OK;
    }

    ZP_Error unlink(const char* path) override {
        if (!path) return ZP_ERROR_NULLPTR;
        if (std::remove(path) != 0) return ZP_ERROR_FAIL | ZP_ERROR_EXT_API;
        return ZP_ERROR_OK;
    }

    ZP_Error rename(const char* path_old, const char* path_new) override {
        if (!path_old || !path_new) return ZP_ERROR_NULLPTR;
        if (std::rename(path_old, path_new) != 0) return ZP_ERROR_FAIL | ZP_ERROR_EXT_API;
        return ZP_ERROR_OK;
    }

    int putc(char c, File* fp) override {
        if (!fp) return -1;
        FileHandle* file = getFileHandle(fp);
        if (!file) return -1;
        return std::fputc(c, file);
    }

    int puts(const char* str, File* cp) override {
        if (!str || !cp) return -1;
        FileHandle* file = getFileHandle(cp);
        if (!file) return -1;
        return std::fputs(str, file);
    }

    int printf(File* fp, const char* str, ...) override {
        if (!fp || !str) return -1;
        FileHandle* file = getFileHandle(fp);
        if (!file) return -1;
        
        va_list args;
        va_start(args, str);
        int result = std::vfprintf(file, str, args);
        va_end(args);
        return result;
    }

    char* gets(char* buff, int len, File* fp) override {
        if (!buff || len <= 0 || !fp) return nullptr;
        FileHandle* file = getFileHandle(fp);
        if (!file) return nullptr;
        return std::fgets(buff, len, file);
    }
    */
};
