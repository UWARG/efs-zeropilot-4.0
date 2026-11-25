#include "sd_manager.hpp"
#include "sd.hpp"

SDManager::SDManager(
    ISystemUtils *systemUtilsDriver,
    IMessageQueue<SdReqMsg> *reqQueue,
    IMessageQueue<SdReqBuf> *bufQueue,
    IMessageQueue<PollResult> *respQueues[static_cast<size_t>(ManagerId_e::NUM_MANAGERS)]
) :
    systemUtilsDriver(systemUtilsDriver),
    requestQueue(reqQueue),
    bufferQueue(bufQueue),
    responseQueues(respQueues),
    profilerId(0) {
       systemUtilsDriver->profilerRegister("SD", &profilerId);
}

void SDManager::sdUpdate(SdReqMsg reqMsg) {
    systemUtilsDriver->profilerBegin(profilerId);

    bool firstMsgRead = false;

    int count;
    requestQueue->count(count);
    count++; // +1 for the reqMsg passed in

    while (count-- > 0) {
        if (firstMsgRead) requestQueue->get(&reqMsg);
        firstMsgRead = true;
        PollResult respMsg;
        respMsg.type = reqMsg.type;

        if (reqMsg.fp == nullptr) {
            respMsg.status |= ZP_ERROR_NULLPTR;
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
                	int bufferQueueCount;
                	bufferQueue->count(bufferQueueCount);
                    if (bufferQueueCount == 0) {
                        respMsg.status |= ZP_ERROR_RESOURCE_UNAVAILABLE; // No buffer available for write operation
                        break;
                    }
                    bufferQueue->get(&writeBuffMsg);
                    uint32_t bytesWritten = 0;
                    respMsg.status |= SDFileSystem::fresultToZpError(f_write(fil, writeBuffMsg.buff, writeBuffMsg.size, reinterpret_cast<UINT*>(&bytesWritten)));
                    if (respMsg.status != ZP_ERROR_OK) {
                        break;
                    }
                    totalSize -= bytesWritten;
                }
                if (totalSize > 0) {
                	respMsg.status |= ZP_ERROR_FAIL;
                }
                respMsg.data.bytesTransferred = reqMsg.totalSize - totalSize;
                if (respMsg.status == ZP_ERROR_OK && reqMsg.type == ReqType_e::WRITE_SYNC) {
                    respMsg.status |= SDFileSystem::fresultToZpError(f_sync(fil));
                }
                break;
            }
            case ReqType_e::SYNC: {
                respMsg.status |= SDFileSystem::fresultToZpError(f_sync(fil));
                break;
            }
            /* TODO: Verify in later PR
            case ReqType_e::LSEEK: {
                respMsg.status |= SDFileSystem::fresultToZpError(f_lseek(fil, static_cast<FSIZE_t>(reqMsg.offset)));
                break;
            }
            case ReqType_e::TELL: {
                respMsg.data.position = f_tell(fil);
                break;
            }
            case ReqType_e::WRITE_SEEK: {
                respMsg.status |= SDFileSystem::fresultToZpError(f_lseek(fil, static_cast<FSIZE_t>(reqMsg.offset)));
                int totalSize = reqMsg.totalSize;
                if (respMsg.status == ZP_ERROR_OK) {
                    SdReqBuf writeBuffMsg;
                	int bufferQueueCount;
                    while (totalSize > 0) {
                    	bufferQueue->count(bufferQueueCount);
                        if (bufferQueueCount == 0) {
                            respMsg.status |= ZP_ERROR_RESOURCE_UNAVAILABLE; // No buffer available for write operation
                            break;
                        }
                        bufferQueue->get(&writeBuffMsg);
                        uint32_t bytesWritten = 0;
                        respMsg.status |= SDFileSystem::fresultToZpError(
                            f_write(fil, writeBuffMsg.buff, writeBuffMsg.size, reinterpret_cast<UINT*>(&bytesWritten)));
                        if (respMsg.status != ZP_ERROR_OK) {
                            break;
                        }
                        totalSize -= bytesWritten;
                    }
                    if (totalSize > 0) {
                    	respMsg.status |= ZP_ERROR_FAIL;
                    }
                    respMsg.data.bytesTransferred = reqMsg.totalSize - totalSize;
                } else {
                	int bufferQueueCount;
                    while (totalSize > 0) {
                    	bufferQueue->count(bufferQueueCount);
                        if (bufferQueueCount == 0) {
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
                respMsg.status |= ZP_ERROR_UNSUPPORTED; // Unknown request type
                break;
            }
        }

        if (reqMsg.sendResp) {
            responseQueues[static_cast<size_t>(reqMsg.id)]->push(&respMsg);
        }
    }

    systemUtilsDriver->profilerEnd(profilerId);
}
