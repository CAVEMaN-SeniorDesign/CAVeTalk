#include <cstddef>

#include <gtest/gtest.h>
#include <gmock/gmock.h>

#include "ooga_booga.pb.h"

#include "cave_talk.h"
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

static CaveTalk_LinkHandle_t kCaveTalk_CTests_LinkHandle = {
    .send = Send,
    .receive = Receive,
    .receive_state = CAVE_TALK_LINK_STATE_RESET,
};

uint8_t buffer[255U] = {0U};

class ListenCallbacksInterface
{
    public:
        virtual ~ListenCallbacksInterface()                                                                                   = 0;
        virtual void HearOogaBooga(const cave_talk_Say ooga_booga)                                                               = 0;
        virtual void HearMovement(const CaveTalk_MetersPerSecond_t speed, const CaveTalk_RadiansPerSecond_t turn_rate) = 0;
        virtual void HearCameraMovement(const CaveTalk_Radian_t pan, const CaveTalk_Radian_t tilt)                     = 0;
        virtual void HearLights(const bool headlights)                                                                 = 0;
        virtual void HearMode(const bool manual)  = 0;
};

void HearOogaBooga(const cave_talk_Say ooga_booga)
{
    if(ooga_booga == cave_talk_Say_SAY_OOGA)
    {
        std::cout << "OOGA RECEIVED" << std::endl;
    }
    else if(ooga_booga == cave_talk_Say_SAY_BOOGA)
    {
        std::cout << "BOOGA RECEIVED" << std::endl;
    }
    else
    {
        std::cout << "BAD OUTPUT" << std::endl;
    }
}

void HearMovement(const CaveTalk_MetersPerSecond_t speed, const CaveTalk_RadiansPerSecond_t turn_rate)
{
    std::cout << "Speed: " << speed << ", and Turn Rate: " << turn_rate << std::endl;
    return;
}

void HearCameraMovement(const CaveTalk_Radian_t pan, const CaveTalk_Radian_t tilt)
{
    std::cout << "Pan: " << pan << ", and Tilt: " << tilt << std::endl;
    return;
}

void HearLights(const bool headlights) 
{
    std::cout << "Headlights: " << headlights << std::endl;
    return;
}

void HearMode(const bool manual)
{
    std::cout << "Manual Control: " << manual << std::endl;
    return;
}

const CaveTalk_ListenCallbacks_t kCaveTalk_ListenCallbacksInterface = {
    .hear_ooga_booga      = HearOogaBooga,
    .hear_movement        = HearMovement,
    .hear_camera_movement = HearCameraMovement,
    .hear_lights          = HearLights,
    .hear_mode            = HearMode,
};



// const CaveTalk_ListenCallbacks_t kCaveTalk_ListenCallbacksStructInterface = {
//     .hear_ooga_booga = ListenCallbacksInterface::HearOogaBooga,
//     .hear_movement = ListenCallbacksInterface::HearMovement,
//     .hear_camera_movement = ListenCallbacksInterface::HearCameraMovement,
//     .hear_lights = ListenCallbacksInterface::HearLights,
//     .hear_mode = ListenCallbacksInterface::HearMode,
// };

ListenCallbacksInterface::~ListenCallbacksInterface() = default;

class MockListenerCallbacks : public ListenCallbacksInterface
{
    public:
        MOCK_METHOD(void, HearOogaBooga, (const cave_talk_Say), (override));
        MOCK_METHOD(void, HearMovement, ((const CaveTalk_MetersPerSecond_t), (const CaveTalk_RadiansPerSecond_t)), (override));
        MOCK_METHOD(void, HearCameraMovement, ((const CaveTalk_Radian_t), (const CaveTalk_Radian_t)), (override));
        MOCK_METHOD(void, HearLights, (const bool), (override));
        MOCK_METHOD(void, HearMode, (const bool), (override));
};

static CaveTalk_Handle_t CaveTalk_Handle = {
    .link_handle = kCaveTalk_CTests_LinkHandle,
    .buffer = buffer,
    .buffer_size = 255U,
    .listen_callbacks = kCaveTalk_ListenCallbacksInterface,
};

TEST(CaveTalkCTests, SpeakListenOogaBooga){


    ring_buffer.Clear();

    ASSERT_EQ(CAVE_TALK_ERROR_NONE, CaveTalk_SpeakOogaBooga(&CaveTalk_Handle, cave_talk_Say_SAY_BOOGA));
    ASSERT_EQ(CAVE_TALK_ERROR_NONE, CaveTalk_Hear(&CaveTalk_Handle));

    ring_buffer.Clear();

    ASSERT_EQ(CAVE_TALK_ERROR_NONE, CaveTalk_SpeakOogaBooga(&CaveTalk_Handle, cave_talk_Say_SAY_OOGA));
    ASSERT_EQ(CAVE_TALK_ERROR_NONE, CaveTalk_Hear(&CaveTalk_Handle));


}

TEST(CaveTalkCTests, SpeakListenMovement)
{
    ring_buffer.Clear();

    ASSERT_EQ(CAVE_TALK_ERROR_NONE, CaveTalk_SpeakMovement(&CaveTalk_Handle, 2.7982, 3.14982));
    ASSERT_EQ(CAVE_TALK_ERROR_NONE, CaveTalk_Hear(&CaveTalk_Handle));

    ring_buffer.Clear();

    ASSERT_EQ(CAVE_TALK_ERROR_NONE, CaveTalk_SpeakMovement(&CaveTalk_Handle, 1.99923, .00784));
    ASSERT_EQ(CAVE_TALK_ERROR_NONE, CaveTalk_Hear(&CaveTalk_Handle));

}

TEST(CaveTalkCTests, SpeakListenCameraMovement)
{

    ring_buffer.Clear();

    ASSERT_EQ(CAVE_TALK_ERROR_NONE, CaveTalk_SpeakCameraMovement(&CaveTalk_Handle, 2.7982, 3.14982));
    ASSERT_EQ(CAVE_TALK_ERROR_NONE, CaveTalk_Hear(&CaveTalk_Handle));

    ring_buffer.Clear();

    ASSERT_EQ(CAVE_TALK_ERROR_NONE, CaveTalk_SpeakCameraMovement(&CaveTalk_Handle, 1.99923, .00784));
    ASSERT_EQ(CAVE_TALK_ERROR_NONE, CaveTalk_Hear(&CaveTalk_Handle));

}

TEST(CaveTalkCTests, SpeakListenLights)
{

    ring_buffer.Clear();

    ASSERT_EQ(CAVE_TALK_ERROR_NONE, CaveTalk_SpeakLights(&CaveTalk_Handle, true));
    ASSERT_EQ(CAVE_TALK_ERROR_NONE, CaveTalk_Hear(&CaveTalk_Handle));

    ring_buffer.Clear();

    ASSERT_EQ(CAVE_TALK_ERROR_NONE, CaveTalk_SpeakLights(&CaveTalk_Handle, false));
    ASSERT_EQ(CAVE_TALK_ERROR_NONE, CaveTalk_Hear(&CaveTalk_Handle));

}

TEST(CaveTalkCTests, SpeakListenMode)
{

    ring_buffer.Clear();

    ASSERT_EQ(CAVE_TALK_ERROR_NONE, CaveTalk_SpeakMode(&CaveTalk_Handle, true));
    ASSERT_EQ(CAVE_TALK_ERROR_NONE, CaveTalk_Hear(&CaveTalk_Handle));

    ring_buffer.Clear();

    ASSERT_EQ(CAVE_TALK_ERROR_NONE, CaveTalk_SpeakMode(&CaveTalk_Handle, false));
    ASSERT_EQ(CAVE_TALK_ERROR_NONE, CaveTalk_Hear(&CaveTalk_Handle));

}