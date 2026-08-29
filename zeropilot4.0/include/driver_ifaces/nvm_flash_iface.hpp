#pragma once

#include "nvm_flash_message.hpp"

class INVMFlash {
protected:
	INVMFlash() = default;
public:
	virtual ~INVMFlash() = default;

	virtual int format() = 0;
	virtual int mount() = 0;
	virtual void init() = 0;

	virtual int write(AbstractMessage *msg) = 0;
	virtual int read(AbstractMessage *msg) = 0;
	virtual int erase(AbstractMessage *msg) = 0;
	virtual int update(AbstractMessage *msg) = 0;
};
