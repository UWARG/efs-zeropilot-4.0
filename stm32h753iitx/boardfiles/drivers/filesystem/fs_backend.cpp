#include "fs_backend.hpp"
#include "sd.hpp"
#include "fatfs.h"

FileStatus FatFsBackend::writeFile(File* fp, const void* buf, uint32_t len, uint32_t* written) {
    FIL* fil = reinterpret_cast<FIL*>(&fp->_storage[0]);
    FRESULT res = f_write(fil, buf, len, reinterpret_cast<UINT*>(written));
    return SDFileSystem::fresultToStatus(res);
}
FileStatus FatFsBackend::syncFile(File* fp) {
    return SDFileSystem::fresultToStatus(f_sync(reinterpret_cast<FIL*>(&fp->_storage[0])));
}
FileStatus FatFsBackend::seekFile(File* fp, uint64_t ofs) {
    return SDFileSystem::fresultToStatus(f_lseek(reinterpret_cast<FIL*>(&fp->_storage[0]), static_cast<FSIZE_t>(ofs)));
}
uint64_t FatFsBackend::tellFile(File* fp) {
    return f_tell(reinterpret_cast<FIL*>(&fp->_storage[0]));   // f_tell can't fail, returns value only
}