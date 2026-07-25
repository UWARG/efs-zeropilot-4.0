#pragma once

#include <gmock/gmock.h>
#include "nvm_flash_iface.hpp"
#include "nvm_flash_message.hpp"

class MockNVMFlash : public INVMFlash {
public:
	MOCK_METHOD(int, format, (), (override));
	MOCK_METHOD(int, mount, (), (override));
	MOCK_METHOD(int, write, (AbstractMessage *msg), (override));
	MOCK_METHOD(int, read, (AbstractMessage *msg), (override));
	MOCK_METHOD(int, erase, (AbstractMessage *msg), (override));
	MOCK_METHOD(int, update, (AbstractMessage *msg), (override));
};
