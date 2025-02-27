#include <cstddef>
#include <functional>

#include <gtest/gtest.h>
#include <gmock/gmock.h>

#include "ooga_booga.pb.h"

#include "cave_talk.h"
#include "cave_talk_link.h"
#include "cave_talk_types.h"
#include "ring_buffer.h"

static const std::size_t kMaxMessageLength = 255U;
static RingBuffer<uint8_t, kMaxMessageLength> ring_buffer;

cave_talk::ListenerCallbacks::~ListenerCallbacks() = default;

class MockListenerCallbacks : public cave_talk::ListenerCallbacks
{
    public:
        MOCK_METHOD(void, HearOogaBooga, (const cave_talk::Say), (override));
        MOCK_METHOD(void, HearMovement, ((const CaveTalk_MetersPerSecond_t), (const CaveTalk_RadiansPerSecond_t)), (override));
        MOCK_METHOD(void, HearCameraMovement, ((const CaveTalk_Radian_t), (const CaveTalk_Radian_t)), (override));
        MOCK_METHOD(void, HearLights, (const bool), (override));
        MOCK_METHOD(void, HearMode, (const bool), (override));
};


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


TEST(CaveTalkCppTests, SpeakListenOogaBooga){

    uint8_t data_receive[255U] = {0U};
    CaveTalk_Id_t id = 0U;
    CaveTalk_Length_t length = 0U;

    std::shared_ptr<MockListenerCallbacks> mock_listen_callbacks = std::make_shared<MockListenerCallbacks>();
    cave_talk::Talker roverMouth(Send);
    cave_talk::Listener roverEars(Receive, Available, mock_listen_callbacks);

    ring_buffer.Clear();

    ASSERT_EQ(CAVE_TALK_ERROR_NONE, roverMouth.SpeakOogaBooga(cave_talk::SAY_OOGA));
    EXPECT_CALL(*mock_listen_callbacks.get(), HearOogaBooga(cave_talk::SAY_OOGA)).Times(1);
    ASSERT_EQ(CAVE_TALK_ERROR_NONE, roverEars.Listen());

    ring_buffer.Clear();

    ASSERT_EQ(CAVE_TALK_ERROR_NONE, roverMouth.SpeakOogaBooga(cave_talk::SAY_BOOGA));
    EXPECT_CALL(*mock_listen_callbacks.get(), HearOogaBooga(cave_talk::SAY_BOOGA)).Times(1);
    ASSERT_EQ(CAVE_TALK_ERROR_NONE, roverEars.Listen());

}

TEST(CaveTalkCppTests, SpeakListenMovement){

    uint8_t data_receive[255U] = {0U};
    CaveTalk_Id_t id = 0U;
    CaveTalk_Length_t length = 0U;

    std::shared_ptr<MockListenerCallbacks> mock_listen_callbacks = std::make_shared<MockListenerCallbacks>();
    cave_talk::Talker roverMouth(Send);
    cave_talk::Listener roverEars(Receive, Available, mock_listen_callbacks);

    ring_buffer.Clear();

    ASSERT_EQ(CAVE_TALK_ERROR_NONE, roverMouth.SpeakMovement(1.0, 2.5));
    EXPECT_CALL(*mock_listen_callbacks.get(), HearMovement(1.0, 2.5)).Times(1);
    ASSERT_EQ(CAVE_TALK_ERROR_NONE, roverEars.Listen());

    ring_buffer.Clear();

    ASSERT_EQ(CAVE_TALK_ERROR_NONE, roverMouth.SpeakMovement(0, 6));
    EXPECT_CALL(*mock_listen_callbacks.get(), HearMovement(0, 6)).Times(1);
    ASSERT_EQ(CAVE_TALK_ERROR_NONE, roverEars.Listen());

    ring_buffer.Clear();

    ASSERT_EQ(CAVE_TALK_ERROR_NONE, roverMouth.SpeakMovement(6.72, 0));
    EXPECT_CALL(*mock_listen_callbacks.get(), HearMovement(6.72, 0)).Times(1);
    ASSERT_EQ(CAVE_TALK_ERROR_NONE, roverEars.Listen());

    ring_buffer.Clear();

    ASSERT_EQ(CAVE_TALK_ERROR_NONE, roverMouth.SpeakMovement(-1.005, -2));
    EXPECT_CALL(*mock_listen_callbacks.get(), HearMovement(-1.005, -2)).Times(1);
    ASSERT_EQ(CAVE_TALK_ERROR_NONE, roverEars.Listen());
    
}

TEST(CaveTalkCppTests, SpeakListenCameraMovement){

    uint8_t data_receive[255U] = {0U};
    CaveTalk_Id_t id = 0U;
    CaveTalk_Length_t length = 0U;

    std::shared_ptr<MockListenerCallbacks> mock_listen_callbacks = std::make_shared<MockListenerCallbacks>();
    cave_talk::Talker roverMouth(Send);
    cave_talk::Listener roverEars(Receive, Available, mock_listen_callbacks);

    ring_buffer.Clear();

    ASSERT_EQ(CAVE_TALK_ERROR_NONE, roverMouth.SpeakCameraMovement(0.0002, 6.28453));
    EXPECT_CALL(*mock_listen_callbacks.get(), HearCameraMovement(0.0002, 6.28453)).Times(1);
    ASSERT_EQ(CAVE_TALK_ERROR_NONE, roverEars.Listen());

    ring_buffer.Clear();

    ASSERT_EQ(CAVE_TALK_ERROR_NONE, roverMouth.SpeakCameraMovement(-200.20, 0));
    EXPECT_CALL(*mock_listen_callbacks.get(), HearCameraMovement(-200.20, 0)).Times(1);
    ASSERT_EQ(CAVE_TALK_ERROR_NONE, roverEars.Listen());

    ring_buffer.Clear();

    ASSERT_EQ(CAVE_TALK_ERROR_NONE, roverMouth.SpeakCameraMovement(1, -6.28453));
    EXPECT_CALL(*mock_listen_callbacks.get(), HearCameraMovement(1, -6.28453)).Times(1);
    ASSERT_EQ(CAVE_TALK_ERROR_NONE, roverEars.Listen());
    
}

TEST(CaveTalkCppTests, SpeakListenLights){

    uint8_t data_receive[255U] = {0U};
    CaveTalk_Id_t id = 0U;
    CaveTalk_Length_t length = 0U;

    std::shared_ptr<MockListenerCallbacks> mock_listen_callbacks = std::make_shared<MockListenerCallbacks>();
    cave_talk::Talker roverMouth(Send);
    cave_talk::Listener roverEars(Receive, Available, mock_listen_callbacks);

    ring_buffer.Clear();

    ASSERT_EQ(CAVE_TALK_ERROR_NONE, roverMouth.SpeakLights(true));
    EXPECT_CALL(*mock_listen_callbacks.get(), HearLights(true)).Times(1);
    ASSERT_EQ(CAVE_TALK_ERROR_NONE, roverEars.Listen());

    ring_buffer.Clear();

    ASSERT_EQ(CAVE_TALK_ERROR_NONE, roverMouth.SpeakLights(false));
    EXPECT_CALL(*mock_listen_callbacks.get(), HearLights(false)).Times(1);
    ASSERT_EQ(CAVE_TALK_ERROR_NONE, roverEars.Listen());
    
}

TEST(CaveTalkCppTests, SpeakListenMode){

    uint8_t data_receive[255U] = {0U};
    CaveTalk_Id_t id = 0U;
    CaveTalk_Length_t length = 0U;

    std::shared_ptr<MockListenerCallbacks> mock_listen_callbacks = std::make_shared<MockListenerCallbacks>();
    cave_talk::Talker roverMouth(Send);
    cave_talk::Listener roverEars(Receive, Available, mock_listen_callbacks);

    ring_buffer.Clear();

    ASSERT_EQ(CAVE_TALK_ERROR_NONE, roverMouth.SpeakMode(true));
    EXPECT_CALL(*mock_listen_callbacks.get(), HearMode(true)).Times(1);
    ASSERT_EQ(CAVE_TALK_ERROR_NONE, roverEars.Listen());

    ring_buffer.Clear();

    ASSERT_EQ(CAVE_TALK_ERROR_NONE, roverMouth.SpeakMode(false));
    EXPECT_CALL(*mock_listen_callbacks.get(), HearMode(false)).Times(1);
    ASSERT_EQ(CAVE_TALK_ERROR_NONE, roverEars.Listen());
    
}