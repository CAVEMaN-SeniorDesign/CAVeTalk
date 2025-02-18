#include "cave_talk.h"

#include <cstddef>
#include <functional>

#include "camera_movement.pb.h"
#include "ids.pb.h"
#include "lights.pb.h"
#include "mode.pb.h"
#include "movement.pb.h"
#include "ooga_booga.pb.h"

#include "cave_talk_link.h"
#include "cave_talk_types.h"

namespace cave_talk
{

Listener::Listener(std::function<CaveTalk_Error_t(void *const data, const size_t size, size_t *const bytes_received)> &receive,
                   std::function<CaveTalk_Error_t(size_t *const bytes)> &available,
                   std::shared_ptr<ListenerCallbacks> listener_callbacks) : listener_callbacks_(listener_callbacks)
{
    link_handle_.send      = nullptr;
    link_handle_.receive   = *receive.target<CaveTalk_Error_t (*)(void *const data, const size_t size, size_t *const bytes_received)>();
    link_handle_.available = *available.target<CaveTalk_Error_t (*)(size_t *const bytes)>();
}

CaveTalk_Error_t Listener::Listen(void)
{
    CaveTalk_Id_t     id     = 0U;
    CaveTalk_Length_t length = 0U;
    CaveTalk_Error_t  error  = CaveTalk_Listen(&link_handle_, &id, buffer_.data(), buffer_.size(), &length);

    if (CAVE_TALK_ERROR_NONE == error)
    {
        switch (static_cast<Id>(id))
        {
        case ID_NONE:
            if (0U != length)
            {
                error = CAVE_TALK_ERROR_ID;
            }
            break;
        case ID_OOGA:
            error = HandleOogaBooga(length);
            break;
        case ID_MOVEMENT:
            error = HandleMovement(length);
            break;
        case ID_CAMERA_MOVEMENT:
            error = HandleCameraMovement(length);
            break;
        case ID_LIGHTS:
            error = HandleLights(length);
            break;
        case ID_MODE:
            error = HandleMode(length);
            break;
        default:
            error = CAVE_TALK_ERROR_ID;
            break;
        }
    }

    return error;
}

CaveTalk_Error_t Listener::HandleOogaBooga(CaveTalk_Length_t length)
{

    OogaBooga ooga_booga_message;

    if (!ooga_booga_message.ParseFromArray(buffer_.data(), length))
    {
        return CAVE_TALK_ERROR_PARSE;
    }

    const Say ooga_booga = ooga_booga_message.ooga_booga();

    listener_callbacks_->HearOogaBooga(ooga_booga);

    return CAVE_TALK_ERROR_NONE;
}

CaveTalk_Error_t Listener::HandleMovement(CaveTalk_Length_t length)
{

    Movement movement_message;

    if (!movement_message.ParseFromArray(buffer_.data(), length))
    {
        return CAVE_TALK_ERROR_PARSE;
    }

    const CaveTalk_MetersPerSecond_t  speed     = movement_message.speed_meters_per_second();
    const CaveTalk_RadiansPerSecond_t turn_rate = movement_message.turn_rate_radians_per_second();

    listener_callbacks_->HearMovement(speed, turn_rate);

    return CAVE_TALK_ERROR_NONE;
}

CaveTalk_Error_t Listener::HandleCameraMovement(CaveTalk_Length_t length)
{

    CameraMovement camera_movement_message;

    if (!camera_movement_message.ParseFromArray(buffer_.data(), length))
    {
        return CAVE_TALK_ERROR_PARSE;
    }

    const CaveTalk_Radian_t pan  = camera_movement_message.pan_angle_radians();
    const CaveTalk_Radian_t tilt = camera_movement_message.tilt_angle_radians();

    listener_callbacks_->HearCameraMovement(pan, tilt);

    return CAVE_TALK_ERROR_NONE;
}

CaveTalk_Error_t Listener::HandleLights(CaveTalk_Length_t length)
{

    Lights lights_message;

    if (!lights_message.ParseFromArray(buffer_.data(), length))
    {
        return CAVE_TALK_ERROR_PARSE;
    }

    const bool headlights = lights_message.headlights();

    listener_callbacks_->HearLights(headlights);

    return CAVE_TALK_ERROR_NONE;
}

CaveTalk_Error_t Listener::HandleMode(CaveTalk_Length_t length)
{

    Mode mode_message;

    if (!mode_message.ParseFromArray(buffer_.data(), length))
    {
        return CAVE_TALK_ERROR_PARSE;
    }

    const bool manual = mode_message.manual();

    listener_callbacks_->HearMode(manual);

    return CAVE_TALK_ERROR_NONE;
}

Talker::Talker(std::function<CaveTalk_Error_t(const void *const data, const size_t size)> &send)
{
    link_handle_.send      = *send.target<CaveTalk_Error_t (*)(const void *const data, const size_t size)>();
    link_handle_.receive   = nullptr;
    link_handle_.available = nullptr;
}

CaveTalk_Error_t Talker::SpeakOogaBooga(const Say ooga_booga)
{
    OogaBooga ooga_booga_message;
    ooga_booga_message.set_ooga_booga(ooga_booga);

    std::size_t length = ooga_booga_message.ByteSizeLong();
    message_buffer_.reserve(length);
    ooga_booga_message.SerializeToArray(message_buffer_.data(), message_buffer_.capacity());

    return CaveTalk_Speak(&link_handle_, static_cast<CaveTalk_Id_t>(ID_OOGA), message_buffer_.data(), length);
}

CaveTalk_Error_t Talker::SpeakMovement(const CaveTalk_MetersPerSecond_t speed, const CaveTalk_RadiansPerSecond_t turn_rate)
{
    Movement movement_message;
    movement_message.set_speed_meters_per_second(speed);
    movement_message.set_turn_rate_radians_per_second(turn_rate);

    std::size_t length = movement_message.ByteSizeLong();
    message_buffer_.reserve(length);
    movement_message.SerializeToArray(message_buffer_.data(), message_buffer_.capacity());

    return CaveTalk_Speak(&link_handle_, static_cast<CaveTalk_Id_t>(ID_MOVEMENT), message_buffer_.data(), length);
}

CaveTalk_Error_t Talker::SpeakCameraMovement(const CaveTalk_Radian_t pan, const CaveTalk_Radian_t tilt)
{
    CameraMovement camera_movement_message;
    camera_movement_message.set_pan_angle_radians(pan);
    camera_movement_message.set_tilt_angle_radians(tilt);

    std::size_t length = camera_movement_message.ByteSizeLong();
    message_buffer_.reserve(length);
    camera_movement_message.SerializeToArray(message_buffer_.data(), message_buffer_.capacity());

    return CaveTalk_Speak(&link_handle_, static_cast<CaveTalk_Id_t>(ID_CAMERA_MOVEMENT), message_buffer_.data(), length);
}

CaveTalk_Error_t Talker::SpeakLights(const bool headlights)
{
    Lights lights_message;
    lights_message.set_headlights(headlights);

    std::size_t length = lights_message.ByteSizeLong();
    message_buffer_.reserve(length);
    lights_message.SerializeToArray(message_buffer_.data(), message_buffer_.capacity());

    return CaveTalk_Speak(&link_handle_, static_cast<CaveTalk_Id_t>(ID_LIGHTS), message_buffer_.data(), length);
}

CaveTalk_Error_t Talker::SpeakMode(const bool manual)
{
    Mode mode_message;
    mode_message.set_manual(manual);

    std::size_t length = mode_message.ByteSizeLong();
    message_buffer_.reserve(length);
    mode_message.SerializeToArray(message_buffer_.data(), message_buffer_.capacity());

    return CaveTalk_Speak(&link_handle_, static_cast<CaveTalk_Id_t>(ID_MODE), message_buffer_.data(), length);
}

} // namespace cave_talk