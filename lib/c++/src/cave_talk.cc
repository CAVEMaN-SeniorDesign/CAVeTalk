#include "cave_talk.h"

#include <cstddef>
#include <functional>

#include "camera_movement.pb.h"
#include "ids.pb.h"
#include "lights.pb.h"
#include "mode.pb.h"
#include "movement.pb.h"
#include "ooga_booga.pb.h"
#include "log.pb.h"
#include "odometry.pb.h"
#include "config_servo.pb.h"
#include "config_motor.pb.h"
#include "cave_talk_link.h"
#include "cave_talk_types.h"

namespace cave_talk
{

Listener::Listener(CaveTalk_Error_t (*receive)(void *const data, const size_t size, size_t *const bytes_received),
                   std::shared_ptr<ListenerCallbacks> listener_callbacks) : listener_callbacks_(listener_callbacks)
{
    link_handle_         = kCaveTalk_LinkHandleNull;
    link_handle_.send    = nullptr;
    link_handle_.receive = receive;
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
        case ID_ODOMETRY:
            error = HandleOdometry(length);
            break;
        case ID_LOG:
            error = HandleLog(length);
            break;
        case ID_CONFIG_SERVO_WHEELS:
            error = HandleConfigServoWheels(length);
            break;
        case ID_CONFIG_SERVO_CAMS:
            error = HandleConfigServoWheels(length);
            break;
        case ID_CONFIG_MOTOR:
            error = HandleConfigMotor(length);
            break;
        default:
            error = CAVE_TALK_ERROR_ID;
            break;
        }
    }

    return error;
}

CaveTalk_Error_t Listener::HandleOogaBooga(CaveTalk_Length_t length) const
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

CaveTalk_Error_t Listener::HandleMovement(CaveTalk_Length_t length) const
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

CaveTalk_Error_t Listener::HandleCameraMovement(CaveTalk_Length_t length) const
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

CaveTalk_Error_t Listener::HandleLights(CaveTalk_Length_t length) const
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

CaveTalk_Error_t Listener::HandleMode(CaveTalk_Length_t length) const
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

CaveTalk_Error_t Listener::HandleLog(CaveTalk_Length_t length) const
{
    Log log_message;

    if (!log_message.ParseFromArray(buffer_.data(), length))
    {
        return CAVE_TALK_ERROR_PARSE;
    }

    const char *const log = (log_message.log_string()).c_str();

    listener_callbacks_->HearLog(log);

    return CAVE_TALK_ERROR_NONE;

}


CaveTalk_Error_t Listener::HandleOdometry(CaveTalk_Length_t length) const
{

    Odometry odometry_message;

    if (!odometry_message.ParseFromArray(buffer_.data(), length))
    {
        return CAVE_TALK_ERROR_PARSE;
    }

    const Imu     IMU             = odometry_message.imu();
    const Encoder encoder_wheel_0 = odometry_message.encoder_wheel_0();
    const Encoder encoder_wheel_1 = odometry_message.encoder_wheel_1();
    const Encoder encoder_wheel_2 = odometry_message.encoder_wheel_2();
    const Encoder encoder_wheel_3 = odometry_message.encoder_wheel_3();

    listener_callbacks_->HearOdometry(IMU, encoder_wheel_0, encoder_wheel_1, encoder_wheel_2, encoder_wheel_3);

    return CAVE_TALK_ERROR_NONE;
}


CaveTalk_Error_t Listener::HandleConfigServoWheels(CaveTalk_Length_t length) const
{
    ConfigServoWheels config_servo_wheels_message;

    if (!config_servo_wheels_message.ParseFromArray(buffer_.data(), length))
    {
        return CAVE_TALK_ERROR_PARSE;
    }

    const Servo servo_wheel_0 = config_servo_wheels_message.servo_wheel_0();
    const Servo servo_wheel_1 = config_servo_wheels_message.servo_wheel_1();
    const Servo servo_wheel_2 = config_servo_wheels_message.servo_wheel_2();
    const Servo servo_wheel_3 = config_servo_wheels_message.servo_wheel_3();

    listener_callbacks_->HearConfigServoWheels(servo_wheel_0, servo_wheel_1, servo_wheel_2, servo_wheel_3);

    return CAVE_TALK_ERROR_NONE;
}

CaveTalk_Error_t Listener::HandleConfigServoCams(CaveTalk_Length_t length) const
{
    ConfigServoCams config_servo_cams_message;

    if (!config_servo_cams_message.ParseFromArray(buffer_.data(), length))
    {
        return CAVE_TALK_ERROR_PARSE;
    }


    const Servo servo_cam_pan  = config_servo_cams_message.servo_cam_pan();
    const Servo servo_cam_tilt = config_servo_cams_message.servo_cam_tilt();

    listener_callbacks_->HearConfigServoCams(servo_cam_pan, servo_cam_tilt);

    return CAVE_TALK_ERROR_NONE;
}

CaveTalk_Error_t Listener::HandleConfigMotor(CaveTalk_Length_t length) const
{
    ConfigMotor config_motor_message;

    if (!config_motor_message.ParseFromArray(buffer_.data(), length))
    {
        return CAVE_TALK_ERROR_PARSE;
    }

    const Motor motor_wheel_0 = config_motor_message.motor_wheel_0();
    const Motor motor_wheel_1 = config_motor_message.motor_wheel_1();
    const Motor motor_wheel_2 = config_motor_message.motor_wheel_2();
    const Motor motor_wheel_3 = config_motor_message.motor_wheel_3();

    listener_callbacks_->HearConfigMotor(motor_wheel_0, motor_wheel_1, motor_wheel_2, motor_wheel_3);

    return CAVE_TALK_ERROR_NONE;
}

Talker::Talker(CaveTalk_Error_t (*send)(const void *const data, const size_t size))
{
    link_handle_         = kCaveTalk_LinkHandleNull;
    link_handle_.send    = send;
    link_handle_.receive = nullptr;
}

CaveTalk_Error_t Talker::SpeakOogaBooga(const Say ooga_booga)
{
    OogaBooga ooga_booga_message;
    ooga_booga_message.set_ooga_booga(ooga_booga);

    std::size_t length = ooga_booga_message.ByteSizeLong();
    ooga_booga_message.SerializeToArray(message_buffer_.data(), message_buffer_.max_size());

    return CaveTalk_Speak(&link_handle_, static_cast<CaveTalk_Id_t>(ID_OOGA), message_buffer_.data(), length);
}

CaveTalk_Error_t Talker::SpeakMovement(const CaveTalk_MetersPerSecond_t speed, const CaveTalk_RadiansPerSecond_t turn_rate)
{
    Movement movement_message;
    movement_message.set_speed_meters_per_second(speed);
    movement_message.set_turn_rate_radians_per_second(turn_rate);

    std::size_t length = movement_message.ByteSizeLong();
    movement_message.SerializeToArray(message_buffer_.data(), message_buffer_.max_size());

    return CaveTalk_Speak(&link_handle_, static_cast<CaveTalk_Id_t>(ID_MOVEMENT), message_buffer_.data(), length);
}

CaveTalk_Error_t Talker::SpeakCameraMovement(const CaveTalk_Radian_t pan, const CaveTalk_Radian_t tilt)
{
    CameraMovement camera_movement_message;
    camera_movement_message.set_pan_angle_radians(pan);
    camera_movement_message.set_tilt_angle_radians(tilt);

    std::size_t length = camera_movement_message.ByteSizeLong();
    camera_movement_message.SerializeToArray(message_buffer_.data(), message_buffer_.max_size());

    return CaveTalk_Speak(&link_handle_, static_cast<CaveTalk_Id_t>(ID_CAMERA_MOVEMENT), message_buffer_.data(), length);
}

CaveTalk_Error_t Talker::SpeakLights(const bool headlights)
{
    Lights lights_message;
    lights_message.set_headlights(headlights);

    std::size_t length = lights_message.ByteSizeLong();
    lights_message.SerializeToArray(message_buffer_.data(), message_buffer_.max_size());

    return CaveTalk_Speak(&link_handle_, static_cast<CaveTalk_Id_t>(ID_LIGHTS), message_buffer_.data(), length);
}

CaveTalk_Error_t Talker::SpeakMode(const bool manual)
{
    Mode mode_message;
    mode_message.set_manual(manual);

    std::size_t length = mode_message.ByteSizeLong();
    mode_message.SerializeToArray(message_buffer_.data(), message_buffer_.max_size());

    return CaveTalk_Speak(&link_handle_, static_cast<CaveTalk_Id_t>(ID_MODE), message_buffer_.data(), length);
}


CaveTalk_Error_t Talker::SpeakLog(const char *const &log)
{
    Log log_message;
    log_message.set_log_string(log);

    std::size_t length = log_message.ByteSizeLong();
    log_message.SerializeToArray(message_buffer_.data(), message_buffer_.max_size());

    return CaveTalk_Speak(&link_handle_, static_cast<CaveTalk_Id_t>(ID_LOG), message_buffer_.data(), length);
}


CaveTalk_Error_t Talker::SpeakOdometry(const Imu &IMU, const Encoder &encoder_wheel_0, const Encoder &encoder_wheel_1, const Encoder &encoder_wheel_2, const Encoder &encoder_wheel_3)
{
    Odometry odometry_message;
    odometry_message.mutable_imu()->CopyFrom(IMU);
    odometry_message.mutable_encoder_wheel_0()->CopyFrom(encoder_wheel_0);
    odometry_message.mutable_encoder_wheel_1()->CopyFrom(encoder_wheel_1);
    odometry_message.mutable_encoder_wheel_2()->CopyFrom(encoder_wheel_2);
    odometry_message.mutable_encoder_wheel_3()->CopyFrom(encoder_wheel_3);

    size_t length = odometry_message.ByteSizeLong();
    odometry_message.SerializeToArray(message_buffer_.data(), message_buffer_.max_size());

    return CaveTalk_Speak(&link_handle_, static_cast<CaveTalk_Id_t>(ID_ODOMETRY), message_buffer_.data(), length);

}


CaveTalk_Error_t Talker::SpeakConfigServoWheels(const Servo &servo_wheel_0, const Servo &servo_wheel_1, const Servo &servo_wheel_2, const Servo &servo_wheel_3)
{
    ConfigServoWheels config_servo_wheels_message;

    config_servo_wheels_message.mutable_servo_wheel_0()->CopyFrom(servo_wheel_0);
    config_servo_wheels_message.mutable_servo_wheel_1()->CopyFrom(servo_wheel_1);
    config_servo_wheels_message.mutable_servo_wheel_2()->CopyFrom(servo_wheel_2);
    config_servo_wheels_message.mutable_servo_wheel_3()->CopyFrom(servo_wheel_3);


    std::size_t length = config_servo_wheels_message.ByteSizeLong();
    config_servo_wheels_message.SerializeToArray(message_buffer_.data(), message_buffer_.max_size());

    return CaveTalk_Speak(&link_handle_, static_cast<CaveTalk_Id_t>(ID_CONFIG_SERVO_WHEELS), message_buffer_.data(), length);
}

CaveTalk_Error_t Talker::SpeakConfigServoCams(const Servo &servo_cam_pan, const Servo &servo_cam_tilt)
{

    ConfigServoCams config_servo_cams_message;

    config_servo_cams_message.mutable_servo_cam_pan()->CopyFrom(servo_cam_pan);
    config_servo_cams_message.mutable_servo_cam_tilt()->CopyFrom(servo_cam_tilt);


    std::size_t length = config_servo_cams_message.ByteSizeLong();
    config_servo_cams_message.SerializeToArray(message_buffer_.data(), message_buffer_.max_size());

    return CaveTalk_Speak(&link_handle_, static_cast<CaveTalk_Id_t>(ID_CONFIG_SERVO_CAMS), message_buffer_.data(), length);

}

CaveTalk_Error_t Talker::SpeakConfigMotor(const Motor &motor_wheel_0, const Motor &motor_wheel_1, const Motor &motor_wheel_2, const Motor &motor_wheel_3)
{
    ConfigMotor config_motor_message;

    config_motor_message.mutable_motor_wheel_0()->CopyFrom(motor_wheel_0);
    config_motor_message.mutable_motor_wheel_1()->CopyFrom(motor_wheel_1);
    config_motor_message.mutable_motor_wheel_2()->CopyFrom(motor_wheel_2);
    config_motor_message.mutable_motor_wheel_3()->CopyFrom(motor_wheel_3);

    std::size_t length = config_motor_message.ByteSizeLong();
    config_motor_message.SerializeToArray(message_buffer_.data(), message_buffer_.max_size());

    return CaveTalk_Speak(&link_handle_, static_cast<CaveTalk_Id_t>(ID_CONFIG_MOTOR), message_buffer_.data(), length);
}

} // namespace cave_talk