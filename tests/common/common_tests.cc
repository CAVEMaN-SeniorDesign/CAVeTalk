#include <cstddef>
#include <cstdint>
#include <functional>

#include <gtest/gtest.h>
#include <gmock/gmock-matchers.h>

#include "cave_talk_link.h"
#include "cave_talk_types.h"
#include "ring_buffer.h"

static const std::size_t kMaxMessageLength = 255U;
static RingBuffer<uint8_t, kMaxMessageLength> ring_buffer;

CaveTalk_Error_t Send(const void *const data, const size_t size)
{    
    CaveTalk_Error_t error = CAVE_TALK_ERROR_NONE;

    if (size > ring_buffer.Capacity() - ring_buffer.Size())
    {
        error = CAVE_TALK_ERROR_INCOMPLETE;
    }
    else
    {
        ring_buffer.Write(static_cast<const uint8_t *const>(data), size);
    }

    return error;
}

CaveTalk_Error_t Receive(void *const data, const size_t size, size_t *const bytes_received)
{
    *bytes_received = ring_buffer.Read(static_cast<uint8_t *const>(data), size);

    return CAVE_TALK_ERROR_NONE;
}

CaveTalk_Error_t SendSocketClosed(const void *const data, const size_t size)
{    
    return CAVE_TALK_ERROR_SOCKET_CLOSED;
}

CaveTalk_Error_t ReceiveSocketClosed(void *const data, const size_t size, size_t *const bytes_received)
{
    return CAVE_TALK_ERROR_SOCKET_CLOSED;
}

CaveTalk_Error_t ReceiveWrongNormal(void *const data, const size_t size, size_t *const bytes_received)
{
    return CAVE_TALK_ERROR_NONE;
}

static CaveTalk_LinkHandle_t LinkHandle = {
    .send      = Send,
    .receive   = Receive,
    .receive_state = CAVE_TALK_LINK_STATE_RESET
};

static CaveTalk_LinkHandle_t NullHandle = {
    .send      = nullptr,
    .receive   = nullptr,
    .receive_state = CAVE_TALK_LINK_STATE_RESET
};

static CaveTalk_LinkHandle_t SocketClosedHandle = {
    .send      = SendSocketClosed,
    .receive   = ReceiveSocketClosed,
    .receive_state = CAVE_TALK_LINK_STATE_RESET
};

TEST(CommonTests, SpeakAndListen)
{
    uint8_t data_send[] = {0xDE, 0xAD, 0xBE, 0xEF};
    uint8_t data_receive[sizeof(data_send)] = {0U};
    CaveTalk_Id_t id = 0U;
    CaveTalk_Length_t length = 0U;

    ring_buffer.Clear();

    ASSERT_EQ(CAVE_TALK_ERROR_NONE, CaveTalk_Speak(&LinkHandle, 0x0F, static_cast<void *>(data_send), sizeof(data_send)));
    ASSERT_EQ(CAVE_TALK_ERROR_NONE, CaveTalk_Listen(&LinkHandle, &id, static_cast<void *>(data_receive), sizeof(data_receive), &length));
    ASSERT_EQ(0x0F, id);
    ASSERT_EQ(sizeof(data_send), length);
    ASSERT_THAT(data_receive, testing::ElementsAreArray(data_send));
}

TEST(CommonTests, NullErrors)
{
    uint8_t data_send[] = {0xDE, 0xAD, 0xBE, 0xEF};
    uint8_t data_receive[sizeof(data_send)] = {0U};
    CaveTalk_Id_t id = 0U;
    CaveTalk_Length_t length = 0U;

    ASSERT_EQ(CAVE_TALK_ERROR_NULL, CaveTalk_Speak(nullptr, 0x0F, static_cast<void *>(data_send), sizeof(data_send)));
    ASSERT_EQ(CAVE_TALK_ERROR_NULL, CaveTalk_Speak(&NullHandle, 0x0F, static_cast<void *>(data_send), sizeof(data_send)));
    ASSERT_EQ(CAVE_TALK_ERROR_NULL, CaveTalk_Speak(&LinkHandle, 0x0F, nullptr, sizeof(data_send)));

    ASSERT_EQ(CAVE_TALK_ERROR_NULL, CaveTalk_Listen(nullptr, &id, static_cast<void *>(data_receive), sizeof(data_receive), &length));
    ASSERT_EQ(CAVE_TALK_ERROR_NULL, CaveTalk_Listen(&NullHandle, &id, static_cast<void *>(data_receive), sizeof(data_receive), &length));
    ASSERT_EQ(CAVE_TALK_ERROR_NULL, CaveTalk_Listen(&LinkHandle, nullptr, static_cast<void *>(data_receive), sizeof(data_receive), &length));
    ASSERT_EQ(CAVE_TALK_ERROR_NULL, CaveTalk_Listen(&LinkHandle, &id, nullptr, sizeof(data_receive), &length));
    ASSERT_EQ(CAVE_TALK_ERROR_NULL, CaveTalk_Listen(&LinkHandle, &id, static_cast<void *>(data_receive), sizeof(data_receive), nullptr));
}

TEST(CommonTests, SRAClosed)
{

    uint8_t data_send[] = {0xDE, 0xAD, 0xBE, 0xEF};
    uint8_t data_receive[sizeof(data_send)] = {0U};
    CaveTalk_Id_t id = 0U;
    CaveTalk_Length_t length = 0U;

    ring_buffer.Clear();

    ASSERT_EQ(CAVE_TALK_ERROR_SOCKET_CLOSED, CaveTalk_Speak(&SocketClosedHandle, 0x0F, static_cast<void *>(data_send), sizeof(data_send)));
    ASSERT_EQ(CAVE_TALK_ERROR_SOCKET_CLOSED, CaveTalk_Listen(&SocketClosedHandle, &id, static_cast<void *>(data_receive), sizeof(data_receive), &length));
}

TEST(CommonTests, SizeIncompleteVersion)
{

    uint8_t data_send[10U] = {0U};
    uint8_t data_receive[3U] = {0U};
    uint8_t data_rand_0[4U] = {0U};
    uint8_t data_rand_1[4U] = {1U, 0U, 0U, 0U};
    CaveTalk_Id_t id = 0U;
    CaveTalk_Length_t length = 0U;

    ASSERT_EQ(CAVE_TALK_ERROR_NONE, CaveTalk_Speak(&LinkHandle, 0x0F, static_cast<void *>(data_send), sizeof(data_send)));
    ASSERT_EQ(CAVE_TALK_ERROR_SIZE, CaveTalk_Listen(&LinkHandle, &id, static_cast<void *>(data_receive), sizeof(data_receive), &length));

    /* TODO SD-182 remove manual reset (size error should automatically result in flushing buffer and reset link handle state) */
    LinkHandle.receive_state = CAVE_TALK_LINK_STATE_RESET;

    ring_buffer.Clear();

    ASSERT_EQ(CAVE_TALK_ERROR_NONE, CaveTalk_Listen(&LinkHandle, &id, static_cast<void *>(data_receive), sizeof(data_receive), &length));
}

TEST(CommonTests, CheckCRC)
{
    /* TODO SD-164 CRC Unit Tests */
}