#pragma once

#include <gmock/gmock.h>
#include "textio_iface.hpp"

class MockTextIO : public ITextIO {
public:
    MOCK_METHOD(int, mountFile, (), (override));
    MOCK_METHOD(int, open, (char *file, int modes), (override));
    MOCK_METHOD(int, close, (), (override));
    MOCK_METHOD(char*, read, (char *buffer, int bufferSize), (override));
    MOCK_METHOD(int, write, (const char *buffer), (override));
    MOCK_METHOD(int, seek, (int offset), (override));
    MOCK_METHOD(uint64_t, fsize, (), (override));
    MOCK_METHOD(uint64_t, tell, (), (override));
    MOCK_METHOD(int, eof, (), (override));
    MOCK_METHOD(bool, checkFileExist, (char *file), (override));
};