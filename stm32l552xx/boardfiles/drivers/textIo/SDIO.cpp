#include "SDIO.hpp"
#include <cstring>

SDIO::SDIO() {
  //blank
}

int SDIO::mountFile() {
    return f_mount(&FatFs, "", 1);
}

int SDIO::open(char *file, int modes) {
    if (f_open(&fil, file, modes) != FR_OK) {
        return 1; // Error opening file
    }
    return 0; // Success
}

int SDIO::close() {
    if (f_close(&fil) != FR_OK) {
        return 1; // Error closing file
    }
    return 0; // Success
}

char* SDIO::read(char *buffer, int bufferSize) {
    return f_gets(buffer, bufferSize, &fil); // Reads a line from the file into the buffer
}

int SDIO::write(const char *buffer) {
    return f_puts(buffer, &fil); // Writes a string to the file
}

int SDIO::seek(int offset) {
    if (f_lseek(&fil, offset) != 0) {
        return 1; // Error seeking in file
    }
    return 0; // Success
}

uint64_t SDIO::fsize() {
  return f_size(&fil);
}

uint64_t SDIO::tell() {
    return f_tell(&fil); // Returns the current position in the file
}

int SDIO::eof() {
    return f_eof(&fil); // Returns whether the end of file has been reached
}

bool SDIO::checkFileExist(char *file) {
    FILINFO fno;
    return (f_stat(file, &fno) == FR_OK);
}