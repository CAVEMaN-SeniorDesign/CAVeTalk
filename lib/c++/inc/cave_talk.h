#ifndef CAVE_TALK_H
#define CAVE_TALK_H

#include <cstddef>
#include <functional>
#include <memory>
#include <vector>

#include "ooga_booga.pb.h"

#include "cave_talk_link.h"
#include "cave_talk_types.h"

namespace cave_talk
{


const std::size_t kMaxPayloadSize = 255;

class ListenerCallbacks
{
    public:
        virtual ~ListenerCallbacks()                                                                                   = 0;
        virtual void HearOogaBooga(const Say ooga_booga)                                                               = 0;
        virtual void HearMovement(const CaveTalk_MetersPerSecond_t speed, const CaveTalk_RadiansPerSecond_t turn_rate) = 0;
        virtual void HearCameraMovement(const CaveTalk_Radian_t pan, const CaveTalk_Radian_t tilt)                     = 0;
        virtual void HearLights(const bool headlights)                                                                 = 0;
        virtual void HearMode(const bool manual)                                                                       = 0;
};

class Listener
{
    public:
        Listener(std::function<CaveTalk_Error_t(void *const data, const size_t size, size_t *const bytes_received)> &receive,
                 std::function<CaveTalk_Error_t(size_t *const bytes_available)> &available,
                 std::shared_ptr<ListenerCallbacks> listener_callbacks);
        Listener(Listener &listener)                  = delete;
        Listener(Listener &&listener)                 = delete;
        Listener &operator=(const Listener &listener) = delete;
        Listener &operator=(Listener &&listener)      = delete;
        CaveTalk_Error_t Listen(void);

    private:
        CaveTalk_Error_t HandleOogaBooga(const CaveTalk_Length_t length);
        CaveTalk_Error_t HandleMovement(const CaveTalk_Length_t length);
        CaveTalk_Error_t HandleCameraMovement(const CaveTalk_Length_t length);
        CaveTalk_Error_t HandleLights(const CaveTalk_Length_t length);
        CaveTalk_Error_t HandleMode(const CaveTalk_Length_t length);
        CaveTalk_LinkHandle_t link_handle_;
        std::shared_ptr<ListenerCallbacks> listener_callbacks_;
        std::array<uint8_t, kMaxPayloadSize> buffer_;
};

class Talker
{
    public:
        explicit Talker(std::function<CaveTalk_Error_t(const void *const data, const size_t size)> &send);
        Talker(Talker &talker)                  = delete;
        Talker(Talker &&talker)                 = delete;
        Talker &operator=(const Talker &talker) = delete;
        Talker &operator=(Talker &&talker)      = delete;
        CaveTalk_Error_t SpeakOogaBooga(const Say ooga_booga);
        CaveTalk_Error_t SpeakMovement(const CaveTalk_MetersPerSecond_t speed, const CaveTalk_RadiansPerSecond_t turn_rate);
        CaveTalk_Error_t SpeakCameraMovement(const CaveTalk_Radian_t pan, const CaveTalk_Radian_t tilt);
        CaveTalk_Error_t SpeakLights(const bool headlights);
        CaveTalk_Error_t SpeakMode(const bool manual);

    private:
        CaveTalk_LinkHandle_t link_handle_;
        std::vector<uint8_t> message_buffer_;
};

} // namespace cave_talk

#endif // CAVE_TALK_H