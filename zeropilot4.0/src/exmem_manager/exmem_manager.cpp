#include "exmem_manager.hpp"

ExMemManager::ExMemManager(
    ISystemUtils *systemUtilsDriver,
    IFileSystemBackend *backend,
    IMessageQueue<ExMemReqMsg> *reqQueue,
    IMessageQueue<ExMemReqBuff> *buffQueue,
    IMessageQueue<PollResult> *respQueues[static_cast<size_t>(ManId::COUNT)]
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
            case ReqType::WRITE:
            case ReqType::WRITE_SYNC: {
                ExMemReqBuff writeBuffMsg;
                int totalSize = reqMsg.total_size;
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
                respMsg.data.bytes_transferred = reqMsg.total_size - totalSize;
                if (respMsg.status == FILE_STATUS_OK && reqMsg.type == ReqType::WRITE_SYNC) {
                    respMsg.status = backend->syncFile(reqMsg.fp);
                }
                break;
            }
            case ReqType::LSEEK: {
                respMsg.status = backend->seekFile(reqMsg.fp, reqMsg.offset);
                break;
            }
            case ReqType::TELL: {
                respMsg.data.position = backend->tellFile(reqMsg.fp);
                respMsg.status = FILE_STATUS_OK; // tell doesn't return a result code
                break;
            }
            case ReqType::WRITE_SEEK: {
                respMsg.status = backend->seekFile(reqMsg.fp, reqMsg.offset);
                int totalSize = reqMsg.total_size;
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
                    respMsg.data.bytes_transferred = reqMsg.total_size - totalSize;
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
            case ReqType::SYNC: {
                respMsg.status = backend->syncFile(reqMsg.fp);
                break;
            }
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
