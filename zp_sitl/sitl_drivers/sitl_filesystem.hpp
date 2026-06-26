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
        return *reinterpret_cast<FileHandle**>(&fp->_storage[0]);
    }
    
    // Helper to set FILE* in File struct
    static void setFileHandle(File* fp, FileHandle* handle) {
        if (!fp) return;
        *reinterpret_cast<FileHandle**>(&fp->_storage[0]) = handle;
    }

public:
    SITL_FileSystem() = default;
    ~SITL_FileSystem() = default;

    FileStatus open(File* fp, const char* path, const char* mode) override {
        if (!fp || !path || !mode) return FILE_STATUS_ERROR;
        
        FileHandle* file = std::fopen(path, mode);
        if (!file) return FILE_STATUS_ERROR;
        
        setFileHandle(fp, file);
        return FILE_STATUS_OK;
    }

    FileStatus close(File* fp) override {
        if (!fp) return FILE_STATUS_ERROR;
        FileHandle* file = getFileHandle(fp);
        if (!file) return FILE_STATUS_ERROR;
        
        if (std::fclose(file) != 0) return FILE_STATUS_ERROR;
        setFileHandle(fp, nullptr);
        return FILE_STATUS_OK;
    }

    FileStatus read(File* fp, void* buff, uint32_t btr, uint32_t* br) override {
        if (!fp || !buff) return FILE_STATUS_ERROR;
        FileHandle* file = getFileHandle(fp);
        if (!file) return FILE_STATUS_ERROR;
        
        size_t bytes_read = std::fread(buff, 1, btr, file);
        if (br) *br = static_cast<uint32_t>(bytes_read);
        
        if (std::ferror(file)) return FILE_STATUS_ERROR;
        return FILE_STATUS_OK;
    }

    FileStatus write(File* fp, const void* buff, uint32_t btw, uint32_t* bw) override {
        if (!fp || !buff) return FILE_STATUS_ERROR;
        FileHandle* file = getFileHandle(fp);
        if (!file) return FILE_STATUS_ERROR;
        
        size_t bytes_written = std::fwrite(buff, 1, btw, file);
        if (bw) *bw = static_cast<uint32_t>(bytes_written);
        
        if (std::ferror(file)) return FILE_STATUS_ERROR;
        return FILE_STATUS_OK;
    }

    FileStatus lseek(File* fp, uint64_t ofs) override {
        if (!fp) return FILE_STATUS_ERROR;
        FileHandle* file = getFileHandle(fp);
        if (!file) return FILE_STATUS_ERROR;
        
        if (std::fseek(file, static_cast<long>(ofs), SEEK_SET) != 0) {
            return FILE_STATUS_ERROR;
        }
        return FILE_STATUS_OK;
    }

    FileStatus truncate(File* fp) override {
        if (!fp) return FILE_STATUS_ERROR;
        FileHandle* file = getFileHandle(fp);
        if (!file) return FILE_STATUS_ERROR;
        
        long pos = std::ftell(file);
        if (pos < 0) return FILE_STATUS_ERROR;
        
        if (PLATFORM_TRUNCATE(file, pos) != 0) return FILE_STATUS_ERROR;
        return FILE_STATUS_OK;
    }

    FileStatus rewind(File* fp) override {
        if (!fp) return FILE_STATUS_ERROR;
        FileHandle* file = getFileHandle(fp);
        if (!file) return FILE_STATUS_ERROR;
        
        std::rewind(file);
        return FILE_STATUS_OK;
    }

    FileStatus tell(File* fp, uint64_t* position) override {
        if (!fp || !position) return FILE_STATUS_ERROR;
        FileHandle* file = getFileHandle(fp);
        if (!file) return FILE_STATUS_ERROR;
        
        long pos = std::ftell(file);
        if (pos < 0) return FILE_STATUS_ERROR;
        
        *position = static_cast<uint64_t>(pos);
        return FILE_STATUS_OK;
    }

    FileStatus sync(File* fp) override {
        if (!fp) return FILE_STATUS_ERROR;
        FileHandle* file = getFileHandle(fp);
        if (!file) return FILE_STATUS_ERROR;
        
        if (std::fflush(file) != 0) return FILE_STATUS_ERROR;
        return FILE_STATUS_OK;
    }

    FileStatus mkdir(const char* path) override {
        if (!path) return FILE_STATUS_ERROR;
        if (PLATFORM_MKDIR(path) != 0) return FILE_STATUS_ERROR;
        return FILE_STATUS_OK;
    }

    FileStatus unlink(const char* path) override {
        if (!path) return FILE_STATUS_ERROR;
        if (std::remove(path) != 0) return FILE_STATUS_ERROR;
        return FILE_STATUS_OK;
    }

    FileStatus rename(const char* path_old, const char* path_new) override {
        if (!path_old || !path_new) return FILE_STATUS_ERROR;
        if (std::rename(path_old, path_new) != 0) return FILE_STATUS_ERROR;
        return FILE_STATUS_OK;
    }

    FileStatus stat(const char* path, FileInfo* fno) override {
        if (!path || !fno) return FILE_STATUS_ERROR;
        
        struct stat st;
        if (::stat(path, &st) != 0) return FILE_STATUS_ERROR;
        
        fno->size = static_cast<uint64_t>(st.st_size);
        fno->isDir = S_ISDIR(st.st_mode) ? 1 : 0;
        std::strncpy(fno->name, path, 255);
        fno->name[255] = '\0';
        
        return FILE_STATUS_OK;
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
};
