#include "fatfs_manager.hpp"

FatFSManager::FatFSManager(MessageQueue<FatFSReqMsg> *reqQueue, MessageQueue<FatFSReqBuff> *buffQueue, MessageQueue<FatFSRespMsg> *respQueues[ManId::COUNT]) 
    : requestQueue(reqQueue), bufferQueue(buffQueue), responseQueues(respQueues) {}

void FatFSManager::run() {
    while (true) {
        FatFSReqMsg reqMsg;
        if (requestQueue->get(&reqMsg) == osOK) {
            PollResult respMsg;
            respMsg.type = reqMsg.type;

            // CURRENTLY NOT ALL POSSIBLE REQUEST TYPES ARE IMPLEMENTED, ONLY SOME FOR MVP
            switch (reqMsg.type) {
                case ReqType::WRITE:
                    FatFSReqBuff writeBuffMsg;
                    int totalSize = reqMsg.data.total_size;
                    while (totalSize > 0) {
                        if (bufferQueue->get(&writeBuffMsg) == osOK) {
                            uint32_t bytesToWrite = writeBuffMsg.size;
                            uint32_t bytesWritten;
                            FRESULT res = f_write(reqMsg.fp, writeBuffMsg.buff, bytesToWrite, &bytesWritten);
                            if (res != FR_OK) {
                                respMsg.status = fresultToStatus(res);
                                break;
                            }
                            totalSize -= bytesWritten;
                        } else {
                            respMsg.status = FILE_STATUS_ERROR; // Failed to get buffer for write operation
                            break;
                        }
                    }
                    respMsg.status = (totalSize == 0) ? FILE_STATUS_OK : FILE_STATUS_ERROR;
                    respMsg.data.bytes_transferred = reqMsg.data.total_size - totalSize;
                    break;
                case ReqType::LSEEK:
                    res = f_lseek(reqMsg.fp, static_cast<FSIZE_t>(reqMsg.data.offset));
                    respMsg.status = fresultToStatus(res);
                    break;
                case ReqType::TELL:
                    respMsg.data.position = f_tell(reqMsg.fp);
                    respMsg.status = FILE_STATUS_OK; // f_tell doesn't return a result code
                    break;
                case ReqType::WRITE_SEEK:
                    res = f_lseek((FIL*)reqMsg.fp->_storage, static_cast<FSIZE_t>(reqMsg.data.offset));
                    int totalSize = reqMsg.data.total_size;
                    if (res == FR_OK) {
                        FatFSReqBuff writeBuffMsg;
                        while (totalSize > 0) {
                            if (bufferQueue->get(&writeBuffMsg) == osOK) {
                                uint32_t bytesToWrite = writeBuffMsg.size;
                                uint32_t bytesWritten;
                                FRESULT res = f_write(reqMsg.fp, writeBuffMsg.buff, bytesToWrite, &bytesWritten);
                                if (res != FR_OK) {
                                    respMsg.status = fresultToStatus(res);
                                    break;
                                }
                                totalSize -= bytesWritten;
                            } else {
                                respMsg.status = FILE_STATUS_ERROR; // Failed to get buffer for write operation
                                break;
                            }
                        }
                        respMsg.status = (totalSize == 0) ? FILE_STATUS_OK : FILE_STATUS_ERROR;
                        respMsg.data.bytes_transferred = reqMsg.data.total_size - totalSize;
                    } else {
                        respMsg.status = fresultToStatus(res);
                        while (totalSize > 0) {
                            FatFSReqBuff dummyBuff;
                            if (bufferQueue->get(&dummyBuff) != osOK) {
                                break;
                            }
                            totalSize -= dummyBuff.size; // Decrease totalSize to eventually clear all related buffers
                        }
                    }
                    break;
                case ReqType::WRITE_SYNC:
                    FatFSReqBuff writeBuffMsg;
                    int totalSize = reqMsg.data.total_size;
                    while (totalSize > 0) {
                        if (bufferQueue->get(&writeBuffMsg) == osOK && writeBuffMsg.id == reqMsg.id && writeBuffMsg.type == reqMsg.type) {
                            uint32_t bytesToWrite = writeBuffMsg.size;
                            uint32_t bytesWritten;
                            FRESULT res = f_write(reqMsg.fp, writeBuffMsg.buff, bytesToWrite, &bytesWritten);
                            if (res != FR_OK) {
                                respMsg.status = fresultToStatus(res);
                                break;
                            }
                            totalSize -= bytesWritten;
                        } else {
                            respMsg.status = FILE_STATUS_ERROR; // Failed to get buffer for write operation
                            break;
                        }
                    }
                    respMsg.status = (totalSize == 0) ? FILE_STATUS_OK : FILE_STATUS_ERROR;
                    respMsg.data.bytes_transferred = reqMsg.data.total_size - totalSize;
                    if (respMsg.status == FILE_STATUS_OK) {
                        respMsg.status = fresultToStatus(f_sync(reqMsg.fp));
                    }
                    break;
                default:
                    respMsg.status = FILE_STATUS_UNKNOWN; // Unknown request type
                    break;
            }

            responseQueues[reqMsg.id]->push(&respMsg);
        }
    }
}