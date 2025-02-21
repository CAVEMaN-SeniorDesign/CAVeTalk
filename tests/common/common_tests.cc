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
    *bytes_available = ring_buffer.Capacity() - ring_buffer.Size();

    return CAVE_TALK_ERROR_NONE;
}

static const CaveTalk_LinkHandle_t kLinkHandle = {
    .send      = Send,
    .receive   = Receive,
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

// TODO SD-215