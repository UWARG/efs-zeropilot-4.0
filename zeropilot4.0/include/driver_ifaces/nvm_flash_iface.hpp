#pragma once
#include <cstdint>

#include "nvm_flash_message.hpp"
#include "filesystem_datatypes.hpp"

#define NVM_QUEUE_PAYLOAD_SIZE 256u

enum class NvmOpType_e : uint8_t {
	WRITE,
	READ,
	ERASE,
	UPDATE
};

struct NvmTxMsg {
	ManagerId_e caller;
	NvmOpType_e opType;
	uint32_t recordId;
	uint8_t payload[NVM_QUEUE_PAYLOAD_SIZE];
	uint16_t len;
};

struct NvmRxMsg {
	NvmOpType_e opType;
	int status;
	uint32_t recordId;
	uint8_t payload[NVM_QUEUE_PAYLOAD_SIZE];
	uint16_t len;
};

class INVMFlash {
protected:
	INVMFlash() = default;
public:
	virtual ~INVMFlash() = default;

	virtual int format() = 0;
	virtual int mount() = 0;
	virtual void init() = 0;

	virtual int enqueue(ManagerId_e caller, NvmOpType_e op, AbstractMessage *msg) = 0;
	virtual bool poll(ManagerId_e caller, NvmRxMsg *out) = 0;
	virtual void ftlUpdate() = 0;

	virtual void test_message() = 0;
};

class NVMFlashHandle {
public:
	NVMFlashHandle(INVMFlash *nvmFlash, ManagerId_e id) : driver(nvmFlash), id(id) {}

	int write(AbstractMessage *msg) {
		return driver->enqueue(id, NvmOpType_e::WRITE, msg);
	}
	int read(AbstractMessage *msg) {
		return driver->enqueue(id, NvmOpType_e::READ, msg);
	}
	int erase(AbstractMessage *msg) {
		return driver->enqueue(id, NvmOpType_e::ERASE, msg);
	}
	int update(AbstractMessage *msg) {
		return driver->enqueue(id, NvmOpType_e::UPDATE, msg);
	}

	bool poll(NvmRxMsg *out) {
		return driver->poll(id, out);
	}
	bool pollInto(int *status, AbstractMessage *msg) {
		NvmRxMsg rxMsg;
		if (!driver->poll(id, &rxMsg)) {
			return false;
		}

		*status = rxMsg.status;
		msg->id = rxMsg.recordId;
		if (status == 0 && rxMsg.len > 0) {
			msg->unpack(rxMsg.payload, rxMsg.len);
		}
		return true;
	}

private:
	INVMFlash *driver;
	ManagerId_e id;
};
