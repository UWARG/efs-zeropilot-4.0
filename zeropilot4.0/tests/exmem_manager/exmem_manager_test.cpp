#include <gtest/gtest.h>
#include <gmock/gmock.h>
#include "exmem_manager.hpp"
#include "mock_filesystem_backend.hpp"
#include "mock_systemutils.hpp"
#include "mock_queue.hpp"

using ::testing::_;
using ::testing::DoAll;
using ::testing::NiceMock;
using ::testing::Return;
using ::testing::SetArgPointee;

class ExmemManagerTest : public ::testing::Test{
protected:
    int logWrites = 0; // Count Logger::log calls regardless of write or writeAndSync

    NiceMock<MockSystemUtils> mockSystemUtils;
    NiceMock<MockFileSystemBackend> mockFileSystemBackend;
    NiceMock<MockMessageQueue<ExMemReqMsg>> mockReqQueue;
    NiceMock<MockMessageQueue<ExMemReqBuf>> mockBufQueue;
    NiceMock<MockMessageQueue<PollResult>> mockRespQueue[static_cast<size_t>(ManagerId_e::NUM_MANAGERS)];
    IMessageQueue<PollResult>* respQueuePtrs[static_cast<size_t>(ManagerId_e::NUM_MANAGERS)];


    void SetUp() override { 
        for (size_t i = 0; i < static_cast<size_t>(ManagerId_e::NUM_MANAGERS); ++i)
            respQueuePtrs[i] = &mockRespQueue[i];

        ON_CALL(mockFileSystemBackend, writeFile(_, _, _, _)).WillByDefault(Return(FILE_STATUS_OK));
        ON_CALL(mockFileSystemBackend, syncFile(_)).WillByDefault(Return(FILE_STATUS_OK));
    }
};

TEST_F(ExmemManagerTest, writeTest) {
    ExMemReqMsg reqMsg;
    reqMsg.id = ManagerId_e::SYSTEM;
    reqMsg.type = ReqType_e::WRITE;
    reqMsg.totalSize = 1;

    ExMemReqBuf buf{};
    buf.size = 1;

    EXPECT_CALL(mockBufQueue, count()).WillRepeatedly(Return(1));
    EXPECT_CALL(mockBufQueue, get(_)).WillRepeatedly(DoAll(SetArgPointee<0>(buf), Return(0)));
    EXPECT_CALL(mockFileSystemBackend, writeFile(_, _, _, _))
        .WillOnce(DoAll(SetArgPointee<3>(1u), Return(FILE_STATUS_OK)));

    PollResult captured{};
    int pushCount = 0;
    EXPECT_CALL(mockRespQueue[static_cast<size_t>(ManagerId_e::SYSTEM)], push(_))
        .WillOnce([&](PollResult* m) { captured = *m; ++pushCount; return 0; });

    ExMemManager em(&mockSystemUtils, &mockFileSystemBackend, &mockReqQueue, &mockBufQueue, respQueuePtrs);
    em.emUpdate(reqMsg);

    EXPECT_EQ(pushCount, 1);
    EXPECT_EQ(captured.status, FILE_STATUS_OK);
    EXPECT_EQ(captured.data.bytesTransferred, 1u);
}

TEST_F(ExmemManagerTest, WriteFailWhenBufEmptyTest) {
    ExMemReqMsg reqMsg;
    reqMsg.id = ManagerId_e::SYSTEM;
    reqMsg.type = ReqType_e::WRITE;
    reqMsg.totalSize = 2;

    EXPECT_CALL(mockBufQueue, count()).WillRepeatedly(Return(0)); // no buffers available
    EXPECT_CALL(mockFileSystemBackend, writeFile(_, _, _, _)).Times(0); // never attempts a write

    PollResult captured{};
    int pushCount = 0;
    EXPECT_CALL(mockRespQueue[static_cast<size_t>(ManagerId_e::SYSTEM)], push(_))
        .WillOnce([&](PollResult* m) { captured = *m; ++pushCount; return 0; });

    ExMemManager em(&mockSystemUtils, &mockFileSystemBackend, &mockReqQueue, &mockBufQueue, respQueuePtrs);
    em.emUpdate(reqMsg);

    EXPECT_EQ(pushCount, 1);
    EXPECT_EQ(captured.status, FILE_STATUS_ERROR);
    EXPECT_EQ(captured.data.bytesTransferred, 0u);
}

TEST_F(ExmemManagerTest, WriteFailWhenBackendWriteFails) {
    ExMemReqMsg reqMsg;
    reqMsg.id = ManagerId_e::SYSTEM;
    reqMsg.type = ReqType_e::WRITE;
    reqMsg.totalSize = 2;

    ExMemReqBuf buf{};
    buf.size = 1; // one buffer only covers part of totalSize, so a success path would loop again

    EXPECT_CALL(mockBufQueue, count())
        .WillOnce(Return(1))
        .WillRepeatedly(Return(0));
    EXPECT_CALL(mockBufQueue, get(_)).WillRepeatedly(DoAll(SetArgPointee<0>(buf), Return(0)));

    EXPECT_CALL(mockFileSystemBackend, writeFile(_, _, _, _))
        .Times(1)
        .WillOnce(Return(FILE_STATUS_ERROR));

    PollResult captured{};
    int pushCount = 0;
    EXPECT_CALL(mockRespQueue[static_cast<size_t>(ManagerId_e::SYSTEM)], push(_))
        .WillOnce([&](PollResult* m) { captured = *m; ++pushCount; return 0; });

    ExMemManager em(&mockSystemUtils, &mockFileSystemBackend, &mockReqQueue, &mockBufQueue, respQueuePtrs);
    em.emUpdate(reqMsg);

    EXPECT_EQ(pushCount, 1);
    EXPECT_EQ(captured.type, ReqType_e::WRITE);
    EXPECT_EQ(captured.status, FILE_STATUS_ERROR);
    EXPECT_EQ(captured.data.bytesTransferred, 0u);
}
