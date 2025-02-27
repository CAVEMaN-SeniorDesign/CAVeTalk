#ifndef CAVE_TALK_H
#define CAVE_TALK_H

#include <cstddef>
#include <functional>
#include <memory>
#include <vector>

#include "ooga_booga.pb.h"
#include "config_servo.pb.h"
#include "config_motor.pb.h"

#include "cave_talk_link.h"
#include "cave_talk_types.h"

namespace cave_talk
{

const std::size_t kMaxPayloadSize = 255;

class ListenerCallbacks
{
    public:
        virtual ~ListenerCallbacks()                                                                                                                       = 0;
        virtual void HearOogaBooga(const Say ooga_booga)                                                                                                   = 0;
        virtual void HearMovement(const CaveTalk_MetersPerSecond_t speed, const CaveTalk_RadiansPerSecond_t turn_rate)                                     = 0;
        virtual void HearCameraMovement(const CaveTalk_Radian_t pan, const CaveTalk_Radian_t tilt)                                                         = 0;
        virtual void HearLights(const bool headlights)                                                                                                     = 0;
        virtual void HearMode(const bool manual)                                                                                                           = 0;
        virtual void HearConfigServoWheels(const Servo &servo_wheel_0, const Servo &servo_wheel_1, const Servo &servo_wheel_2, const Servo &servo_wheel_3) = 0;
        virtual void HearConfigServoCams(const Servo &servo_cam_pan, const Servo &servo_cam_tilt)                                                          = 0;
        virtual void HearConfigMotor(const Motor &motor_wheel_0, const Motor &motor_wheel_1, const Motor &motor_wheel_2, const Motor &motor_wheel_3)       = 0;
};

class Listener
{
    public:
        Listener(CaveTalk_Error_t (*receive)(void *const data, const size_t size, size_t *const bytes_received),
                 std::shared_ptr<ListenerCallbacks> listener_callbacks);
        Listener(Listener &listener)                  = delete;
        Listener(Listener &&listener)                 = delete;
        Listener &operator=(const Listener &listener) = delete;
        Listener &operator=(Listener &&listener)      = delete;
        CaveTalk_Error_t Listen(void);

    private:
        CaveTalk_Error_t HandleOogaBooga(const CaveTalk_Length_t length) const;
        CaveTalk_Error_t HandleMovement(const CaveTalk_Length_t length) const;
        CaveTalk_Error_t HandleCameraMovement(const CaveTalk_Length_t length) const;
        CaveTalk_Error_t HandleLights(const CaveTalk_Length_t length) const;
        CaveTalk_Error_t HandleMode(const CaveTalk_Length_t length) const;
        CaveTalk_Error_t HandleConfigServoWheels(const CaveTalk_Length_t length) const;
        CaveTalk_Error_t HandleConfigServoCams(const CaveTalk_Length_t length) const;
        CaveTalk_Error_t HandleConfigMotor(const CaveTalk_Length_t length) const;
        CaveTalk_LinkHandle_t link_handle_;
        std::shared_ptr<ListenerCallbacks> listener_callbacks_;
        std::array<uint8_t, kMaxPayloadSize> buffer_;
};

class Talker
{
    public:
        explicit Talker(CaveTalk_Error_t (*send)(const void *const data, const size_t size));
        Talker(Talker &talker)                  = delete;
        Talker(Talker &&talker)                 = delete;
        Talker &operator=(const Talker &talker) = delete;
        Talker &operator=(Talker &&talker)      = delete;
        CaveTalk_Error_t SpeakOogaBooga(const Say ooga_booga);
        CaveTalk_Error_t SpeakMovement(const CaveTalk_MetersPerSecond_t speed, const CaveTalk_RadiansPerSecond_t turn_rate);
        CaveTalk_Error_t SpeakCameraMovement(const CaveTalk_Radian_t pan, const CaveTalk_Radian_t tilt);
        CaveTalk_Error_t SpeakLights(const bool headlights);
        CaveTalk_Error_t SpeakMode(const bool manual);
        CaveTalk_Error_t SpeakConfigServoWheels(const Servo &servo_wheel_0, const Servo &servo_wheel_1, const Servo &servo_wheel_2, const Servo &servo_wheel_3);
        CaveTalk_Error_t SpeakConfigServoCams(const Servo &servo_cam_pan, const Servo &servo_cam_tilt);
        CaveTalk_Error_t SpeakConfigMotor(const Motor &motor_wheel_0, const Motor &motor_wheel_1, const Motor &motor_wheel_2, const Motor &motor_wheel_3);

    private:
        CaveTalk_LinkHandle_t link_handle_;
        std::array<uint8_t, kMaxPayloadSize> message_buffer_;
};

} // namespace cave_talk

#endif // CAVE_TALK_H