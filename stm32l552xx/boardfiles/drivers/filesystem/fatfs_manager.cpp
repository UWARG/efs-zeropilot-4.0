#include "fatfs_manager.hpp"
#include "sd.hpp"

FatFSManager::FatFSManager(MessageQueue<FatFSReqMsg> *reqQueue, MessageQueue<FatFSReqBuff> *buffQueue, MessageQueue<PollResult> *respQueues[static_cast<size_t>(ManId::COUNT)]) 
    : requestQueue(reqQueue), bufferQueue(buffQueue), responseQueues(respQueues) {}

void FatFSManager::run() {
    int count = requestQueue->count();

    while (count-- > 0) {
        FatFSReqMsg reqMsg;
        requestQueue->get(&reqMsg);
        PollResult respMsg;
        respMsg.type = reqMsg.type;

        switch (reqMsg.type) {
            case ReqType::WRITE:
            case ReqType::WRITE_SYNC: {
                FatFSReqBuff writeBuffMsg;
                int totalSize = reqMsg.total_size;
                while (totalSize > 0) {
                    if (bufferQueue->get(&writeBuffMsg) == osOK) {
                        uint32_t bytesToWrite = writeBuffMsg.size;
                        uint32_t bytesWritten;
                        FRESULT res = f_write(reqMsg.fp, writeBuffMsg.buff, bytesToWrite, reinterpret_cast<UINT*>(&bytesWritten));
                        respMsg.status = SDFileSystem::fresultToStatus(res);
                        if (res != FR_OK) {
                            break;
                        }
                        totalSize -= bytesWritten;
                    } else {
                        respMsg.status = FILE_STATUS_ERROR; // Failed to get buffer for write operation
                        break;
                    }
                }
                respMsg.status = (respMsg.status == FILE_STATUS_OK && totalSize <= 0) ? FILE_STATUS_OK : FILE_STATUS_ERROR;
                respMsg.data.bytes_transferred = reqMsg.total_size - totalSize;
                if (respMsg.status == FILE_STATUS_OK && reqMsg.type == ReqType::WRITE_SYNC) {
                    respMsg.status = SDFileSystem::fresultToStatus(f_sync(reqMsg.fp));
                }
                break;
            }
            case ReqType::LSEEK: {
                FRESULT res = f_lseek(reqMsg.fp, static_cast<FSIZE_t>(reqMsg.offset));
                respMsg.status = SDFileSystem::fresultToStatus(res);
                break;
            }
            case ReqType::TELL: {
                respMsg.data.position = f_tell(reqMsg.fp);
                respMsg.status = FILE_STATUS_OK; // f_tell doesn't return a result code
                break;
            }
            case ReqType::WRITE_SEEK: {
                FRESULT res = f_lseek(reqMsg.fp, static_cast<FSIZE_t>(reqMsg.offset));
                int totalSize = reqMsg.total_size;
                if (res == FR_OK) {
                    FatFSReqBuff writeBuffMsg;
                    while (totalSize > 0) {
                        if (bufferQueue->get(&writeBuffMsg) == osOK) {
                            uint32_t bytesToWrite = writeBuffMsg.size;
                            uint32_t bytesWritten;
                            FRESULT res = f_write(reqMsg.fp, writeBuffMsg.buff, bytesToWrite, reinterpret_cast<UINT*>(&bytesWritten));
                            respMsg.status = SDFileSystem::fresultToStatus(res);
                            if (res != FR_OK) {
                                break;
                            }
                            totalSize -= bytesWritten;
                        } else {
                            respMsg.status = FILE_STATUS_ERROR; // Failed to get buffer for write operation
                            break;
                        }
                    }
                    respMsg.status = (respMsg.status == FILE_STATUS_OK && totalSize <= 0) ? FILE_STATUS_OK : FILE_STATUS_ERROR;
                    respMsg.data.bytes_transferred = reqMsg.total_size - totalSize;
                } else {
                    respMsg.status = SDFileSystem::fresultToStatus(res);
                    while (totalSize > 0) {
                        FatFSReqBuff dummyBuff;
                        if (bufferQueue->get(&dummyBuff) != osOK) {
                            break;
                        }
                        totalSize -= dummyBuff.size; // Decrease totalSize to eventually clear all related buffers
                    }
                }
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
}