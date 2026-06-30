#include "exmem_manager.hpp"

ExMemManager::ExMemManager(
    ISystemUtils *systemUtilsDriver,
    IFileSystemBackend *backend,
    IMessageQueue<ExMemReqMsg> *reqQueue,
    IMessageQueue<ExMemReqBuff> *buffQueue,
    IMessageQueue<PollResult> *respQueues[static_cast<size_t>(ManId_e::COUNT)]
) :
    systemUtilsDriver(systemUtilsDriver),
    backend(backend),
    requestQueue(reqQueue),
    bufferQueue(buffQueue),
    responseQueues(respQueues),
    profilerId(0) {
       systemUtilsDriver->profilerRegister("EM", &profilerId);
}

void ExMemManager::emUpdate(ExMemReqMsg reqMsg) {
    systemUtilsDriver->profilerBegin(profilerId);

    bool firstMsgRead = false;

    int count = requestQueue->count() + 1; // +1 for the reqMsg passed in

    while (count-- > 0) {
        if (firstMsgRead) requestQueue->get(&reqMsg);
        firstMsgRead = true;
        PollResult respMsg;
        respMsg.type = reqMsg.type;

        switch (reqMsg.type) {
            case ReqType_e::WRITE:
            case ReqType_e::WRITE_SYNC: {
                ExMemReqBuff writeBuffMsg;
                int totalSize = reqMsg.totalSize;
                while (totalSize > 0) {
                    if (bufferQueue->count() == 0) {
                        respMsg.status = FILE_STATUS_ERROR; // No buffer available for write operation
                        break;
                    }
                    bufferQueue->get(&writeBuffMsg);
                    uint32_t bytesWritten = 0;
                    respMsg.status = backend->writeFile(reqMsg.fp, writeBuffMsg.buff, writeBuffMsg.size, &bytesWritten);
                    if (respMsg.status != FILE_STATUS_OK) {
                        break;
                    }
                    totalSize -= bytesWritten;
                }
                respMsg.status = (respMsg.status == FILE_STATUS_OK && totalSize <= 0) ? FILE_STATUS_OK : FILE_STATUS_ERROR;
                respMsg.data.bytesTransferred = reqMsg.totalSize - totalSize;
                if (respMsg.status == FILE_STATUS_OK && reqMsg.type == ReqType_e::WRITE_SYNC) {
                    respMsg.status = backend->syncFile(reqMsg.fp);
                }
                break;
            }
            case ReqType_e::SYNC: {
                respMsg.status = backend->syncFile(reqMsg.fp);
                break;
            }
            /* TODO: Verify in later PR
            case ReqType_e::LSEEK: {
                respMsg.status = backend->seekFile(reqMsg.fp, reqMsg.offset);
                break;
            }
            case ReqType_e::TELL: {
                respMsg.data.position = backend->tellFile(reqMsg.fp);
                respMsg.status = FILE_STATUS_OK; // tell doesn't return a result code
                break;
            }
            case ReqType_e::WRITE_SEEK: {
                respMsg.status = backend->seekFile(reqMsg.fp, reqMsg.offset);
                int totalSize = reqMsg.totalSize;
                if (respMsg.status == FILE_STATUS_OK) {
                    ExMemReqBuff writeBuffMsg;
                    while (totalSize > 0) {
                        if (bufferQueue->count() == 0) {
                            respMsg.status = FILE_STATUS_ERROR; // No buffer available for write operation
                            break;
                        }
                        bufferQueue->get(&writeBuffMsg);
                        uint32_t bytesWritten = 0;
                        respMsg.status = backend->writeFile(reqMsg.fp, writeBuffMsg.buff, writeBuffMsg.size, &bytesWritten);
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
                        ExMemReqBuff dummyBuff;
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
