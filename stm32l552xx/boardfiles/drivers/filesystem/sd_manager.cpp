#include "sd_manager.hpp"
#include "sd.hpp"

SDManager::SDManager(
    ISystemUtils *systemUtilsDriver,
    IMessageQueue<SdReqMsg> *reqQueue,
    IMessageQueue<SdReqBuf> *bufQueue,
    IMessageQueue<PollResult> *respQueues[static_cast<size_t>(ManagerId_e::NUM_MANAGERS)],
	INVMFlash *nvmDriver
) :
    systemUtilsDriver(systemUtilsDriver),
    requestQueue(reqQueue),
    bufferQueue(bufQueue),
    responseQueues(respQueues),
	nvmDriver(nvmDriver),
    profilerId(0) {
       systemUtilsDriver->profilerRegister("SD", &profilerId);
}

void SDManager::sdUpdate(SdReqMsg reqMsg) {
	nvmDriver->test_message();
    systemUtilsDriver->profilerBegin(profilerId);

    bool firstMsgRead = false;

    int count = requestQueue->count() + 1; // +1 for the reqMsg passed in

    while (count-- > 0) {
        if (firstMsgRead) requestQueue->get(&reqMsg);
        firstMsgRead = true;
        PollResult respMsg;
        respMsg.type = reqMsg.type;

        if (reqMsg.fp == nullptr) {
            respMsg.status = FILE_STATUS_ERROR;
            if (reqMsg.sendResp) {
                responseQueues[static_cast<size_t>(reqMsg.id)]->push(&respMsg);
            }
            continue;
        }

        FIL* fil = reinterpret_cast<FIL*>(&reqMsg.fp->storage[0]);

        switch (reqMsg.type) {
            case ReqType_e::WRITE:
            case ReqType_e::WRITE_SYNC: {
                SdReqBuf writeBuffMsg;
                int totalSize = reqMsg.totalSize;
                while (totalSize > 0) {
                    if (bufferQueue->count() == 0) {
                        respMsg.status = FILE_STATUS_ERROR; // No buffer available for write operation
                        break;
                    }
                    bufferQueue->get(&writeBuffMsg);
                    uint32_t bytesWritten = 0;
                    respMsg.status = SDFileSystem::fresultToStatus(f_write(fil, writeBuffMsg.buff, writeBuffMsg.size, reinterpret_cast<UINT*>(&bytesWritten)));
                    if (respMsg.status != FILE_STATUS_OK) {
                        break;
                    }
                    totalSize -= bytesWritten;
                }
                respMsg.status = (respMsg.status == FILE_STATUS_OK && totalSize <= 0) ? FILE_STATUS_OK : FILE_STATUS_ERROR;
                respMsg.data.bytesTransferred = reqMsg.totalSize - totalSize;
                if (respMsg.status == FILE_STATUS_OK && reqMsg.type == ReqType_e::WRITE_SYNC) {
                    respMsg.status = SDFileSystem::fresultToStatus(f_sync(fil));
                }
                break;
            }
            case ReqType_e::SYNC: {
                respMsg.status = SDFileSystem::fresultToStatus(f_sync(fil));
                break;
            }
            /* TODO: Verify in later PR
            case ReqType_e::LSEEK: {
                respMsg.status = SDFileSystem::fresultToStatus(f_lseek(fil, static_cast<FSIZE_t>(reqMsg.offset)));
                break;
            }
            case ReqType_e::TELL: {
                respMsg.data.position = f_tell(fil);
                respMsg.status = FILE_STATUS_OK; // f_tell doesn't return a result code
                break;
            }
            case ReqType_e::WRITE_SEEK: {
                respMsg.status = SDFileSystem::fresultToStatus(f_lseek(fil, static_cast<FSIZE_t>(reqMsg.offset)));
                int totalSize = reqMsg.totalSize;
                if (respMsg.status == FILE_STATUS_OK) {
                    SdReqBuf writeBuffMsg;
                    while (totalSize > 0) {
                        if (bufferQueue->count() == 0) {
                            respMsg.status = FILE_STATUS_ERROR; // No buffer available for write operation
                            break;
                        }
                        bufferQueue->get(&writeBuffMsg);
                        uint32_t bytesWritten = 0;
                        respMsg.status = SDFileSystem::fresultToStatus(
                            f_write(fil, writeBuffMsg.buff, writeBuffMsg.size, reinterpret_cast<UINT*>(&bytesWritten)));
                        if (respMsg.status != FILE_STATUS_OK) {
                            break;
                        }
                        totalSize -= bytesWritten;
                    }
                    respMsg.status = (respMsg.status == FILE_STATUS_OK && totalSize <= 0) ? FILE_STATUS_OK : FILE_STATUS_ERROR;
                    respMsg.data.bytesTransferred = reqMsg.totalSize - totalSize;
                } else {
                    while (totalSize > 0) {
                        if (bufferQueue->count() == 0) {
                            break;
                        }
                        SdReqBuf dummyBuff;
                        bufferQueue->get(&dummyBuff);
                        totalSize -= dummyBuff.size; // Decrease totalSize to eventually clear all related buffers
                    }
                }
                break;
            }
            */
            default: {
                respMsg.status = FILE_STATUS_UNKNOWN; // Unknown request type
                break;
            }
        }

        if (reqMsg.sendResp) {
            responseQueues[static_cast<size_t>(reqMsg.id)]->push(&respMsg);
        }
    }

    systemUtilsDriver->profilerEnd(profilerId);
}
