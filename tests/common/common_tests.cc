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

CaveTalk_Error_t Available(size_t *const bytes_available)
{
    *bytes_available = ring_buffer.Size();

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

CaveTalk_Error_t AvailableSocketClosed(size_t *const bytes_available)
{
    return CAVE_TALK_ERROR_SOCKET_CLOSED;

}

CaveTalk_Error_t ReceiveWrongNormal(void *const data, const size_t size, size_t *const bytes_received)
{
    return CAVE_TALK_ERROR_NONE;
}

static const CaveTalk_LinkHandle_t kLinkHandle = {
    .send      = Send,
    .receive   = Receive,
    .available = Available,
};

static const CaveTalk_LinkHandle_t kNullHandle = {
    .send      = nullptr,
    .receive   = nullptr,
    .available = nullptr,
};

static const CaveTalk_LinkHandle_t kSocketClosedHandle = {
    .send      = SendSocketClosed,
    .receive   = ReceiveSocketClosed,
    .available = AvailableSocketClosed,
};

static const CaveTalk_LinkHandle_t kAvailHandle = {
    .send      = SendSocketClosed,
    .receive   = ReceiveWrongNormal,
    .available = Available,
};

TEST(CommonTests, SpeakAndListen)
{
    uint8_t data_send[] = {0xDE, 0xAD, 0xBE, 0xEF};
    uint8_t data_receive[sizeof(data_send)] = {0U};
    CaveTalk_Id_t id = 0U;
    CaveTalk_Length_t length = 0U;

    ring_buffer.Clear();

    ASSERT_EQ(CAVE_TALK_ERROR_NONE, CaveTalk_Speak(&kLinkHandle, 0x0F, static_cast<void *>(data_send), sizeof(data_send)));
    ASSERT_EQ(CAVE_TALK_ERROR_NONE, CaveTalk_Listen(&kLinkHandle, &id, static_cast<void *>(data_receive), sizeof(data_receive), &length));
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
    ASSERT_EQ(CAVE_TALK_ERROR_NULL, CaveTalk_Speak(&kNullHandle, 0x0F, static_cast<void *>(data_send), sizeof(data_send)));
    ASSERT_EQ(CAVE_TALK_ERROR_NULL, CaveTalk_Speak(&kLinkHandle, 0x0F, nullptr, sizeof(data_send)));

    ASSERT_EQ(CAVE_TALK_ERROR_NULL, CaveTalk_Listen(nullptr, &id, static_cast<void *>(data_receive), sizeof(data_receive), &length));
    ASSERT_EQ(CAVE_TALK_ERROR_NULL, CaveTalk_Listen(&kNullHandle, &id, static_cast<void *>(data_receive), sizeof(data_receive), &length));
    ASSERT_EQ(CAVE_TALK_ERROR_NULL, CaveTalk_Listen(&kLinkHandle, nullptr, static_cast<void *>(data_receive), sizeof(data_receive), &length));
    ASSERT_EQ(CAVE_TALK_ERROR_NULL, CaveTalk_Listen(&kLinkHandle, &id, nullptr, sizeof(data_receive), &length));
    ASSERT_EQ(CAVE_TALK_ERROR_NULL, CaveTalk_Listen(&kLinkHandle, &id, static_cast<void *>(data_receive), sizeof(data_receive), nullptr));

}

TEST(CommonTests, SRAClosed)
{

    uint8_t data_send[] = {0xDE, 0xAD, 0xBE, 0xEF};
    uint8_t data_receive[sizeof(data_send)] = {0U};
    CaveTalk_Id_t id = 0U;
    CaveTalk_Length_t length = 0U;

    ring_buffer.Clear();

    ASSERT_EQ(CAVE_TALK_ERROR_SOCKET_CLOSED, CaveTalk_Speak(&kSocketClosedHandle, 0x0F, static_cast<void *>(data_send), sizeof(data_send)));
    ASSERT_EQ(CAVE_TALK_ERROR_SOCKET_CLOSED, CaveTalk_Listen(&kSocketClosedHandle, &id, static_cast<void *>(data_receive), sizeof(data_receive), &length));

}

TEST(CommonTests, SizeAndIncompleteness)
{

    uint8_t data_send[10U] = {0U};
    uint8_t data_receive[3U] = {0U};
    uint8_t data_rand_0[4U] = {0U};
    CaveTalk_Id_t id = 0U;
    CaveTalk_Length_t length = 0U;

    ASSERT_EQ(CAVE_TALK_ERROR_NONE, CaveTalk_Speak(&kLinkHandle, 0x0F, static_cast<void *>(data_send), sizeof(data_send)));
    ASSERT_EQ(CAVE_TALK_ERROR_SIZE, CaveTalk_Listen(&kLinkHandle, &id, static_cast<void *>(data_receive), sizeof(data_receive), &length));

    ring_buffer.Clear();

    ASSERT_EQ(CAVE_TALK_ERROR_NONE, CaveTalk_Listen(&kLinkHandle, &id, static_cast<void *>(data_receive), sizeof(data_receive), &length));


    ring_buffer.Write(data_rand_0, sizeof(data_rand_0));

    ASSERT_EQ(CAVE_TALK_ERROR_INCOMPLETE, CaveTalk_Listen(&kLinkHandle, &id, static_cast<void *>(data_receive), sizeof(data_receive), &length));

    ring_buffer.Write(data_rand_0, sizeof(data_rand_0));

    ASSERT_EQ(CAVE_TALK_ERROR_INCOMPLETE, CaveTalk_Listen(&kAvailHandle, &id, static_cast<void *>(data_receive), sizeof(data_receive), &length));


}

TEST(CommonTests, CheckCRC)
{
    /* TODO SD-164 CRC Unit Tests */
}