#include "cave_talk.h"

#include <stdbool.h>

#include "camera_movement.pb.h"
#include "ids.pb.h"
#include "lights.pb.h"
#include "mode.pb.h"
#include "movement.pb.h"
#include "ooga_booga.pb.h"
#include "config_servo.pb.h"
#include "config_motor.pb.h"
#include "pb_decode.h"
#include "pb_encode.h"

#include "cave_talk_link.h"
#include "cave_talk_types.h"


static CaveTalk_Error_t CaveTalk_HandleOogaBooga(const CaveTalk_Handle_t *const handle, const CaveTalk_Length_t length);
static CaveTalk_Error_t CaveTalk_HandleMovement(const CaveTalk_Handle_t *const handle, const CaveTalk_Length_t length);
static CaveTalk_Error_t CaveTalk_HandleCameraMovement(const CaveTalk_Handle_t *const handle, const CaveTalk_Length_t length);
static CaveTalk_Error_t CaveTalk_HandleLights(const CaveTalk_Handle_t *const handle, const CaveTalk_Length_t length);
static CaveTalk_Error_t CaveTalk_HandleMode(const CaveTalk_Handle_t *const handle, const CaveTalk_Length_t length);
static CaveTalk_Error_t CaveTalk_HandleConfigServoWheels(const CaveTalk_Handle_t *const handle, const CaveTalk_Length_t length);
static CaveTalk_Error_t CaveTalk_HandleConfigServoCams(const CaveTalk_Handle_t *const handle, const CaveTalk_Length_t length);
static CaveTalk_Error_t CaveTalk_HandleConfigMotor(const CaveTalk_Handle_t *const handle, const CaveTalk_Length_t length);

CaveTalk_Error_t CaveTalk_Hear(CaveTalk_Handle_t *const handle)
{
    CaveTalk_Error_t error = CAVE_TALK_ERROR_NULL;

    if ((NULL == handle) ||
        (NULL == handle->buffer) ||
        (NULL == handle->link_handle.receive))
    {
    }
    else
    {
        CaveTalk_Id_t     id     = 0U;
        CaveTalk_Length_t length = 0U;

        error = CaveTalk_Listen(&handle->link_handle, &id, handle->buffer, handle->buffer_size, &length);

        if (CAVE_TALK_ERROR_NONE == error)
        {
            switch ((cave_talk_Id)id)
            {
            case cave_talk_Id_ID_NONE:
                if (0U != length)
                {
                    error = CAVE_TALK_ERROR_ID;
                }
                break;
            case cave_talk_Id_ID_OOGA:
                error = CaveTalk_HandleOogaBooga(handle, length);
                break;
            case cave_talk_Id_ID_MOVEMENT:
                error = CaveTalk_HandleMovement(handle, length);
                break;
            case cave_talk_Id_ID_CAMERA_MOVEMENT:
                error = CaveTalk_HandleCameraMovement(handle, length);
                break;
            case cave_talk_Id_ID_LIGHTS:
                error = CaveTalk_HandleLights(handle, length);
                break;
            case cave_talk_Id_ID_MODE:
                error = CaveTalk_HandleMode(handle, length);
                break;
            case cave_talk_Id_ID_CONFIG_SERVO_WHEELS:
                error = CaveTalk_HandleConfigServoWheels(handle, length);
                break;
            case cave_talk_Id_ID_CONFIG_SERVO_CAMS:
                error = CaveTalk_HandleConfigServoCams(handle, length);
                break;
            case cave_talk_Id_ID_CONFIG_MOTOR:
                error = CaveTalk_HandleConfigMotor(handle, length);
                break;

            default:
                error = CAVE_TALK_ERROR_ID;
                break;
            }
        }
    }

    return error;
}

CaveTalk_Error_t CaveTalk_SpeakOogaBooga(const CaveTalk_Handle_t *const handle, const cave_talk_Say ooga_booga)
{
    CaveTalk_Error_t error = CAVE_TALK_ERROR_NULL;

    if ((NULL == handle) || (NULL == handle->buffer) || (NULL == handle->link_handle.send))
    {
    }
    else
    {
        pb_ostream_t        ostream            = pb_ostream_from_buffer(handle->buffer, handle->buffer_size);
        cave_talk_OogaBooga ooga_booga_message = cave_talk_OogaBooga_init_zero;

        ooga_booga_message.ooga_booga = ooga_booga;

        if (!pb_encode(&ostream, cave_talk_OogaBooga_fields, &ooga_booga_message))
        {
            error = CAVE_TALK_ERROR_SIZE;
        }
        else
        {
            error = CaveTalk_Speak(&handle->link_handle, (CaveTalk_Id_t)cave_talk_Id_ID_OOGA, handle->buffer, ostream.bytes_written);
        }
    }

    return error;
}

CaveTalk_Error_t CaveTalk_SpeakMovement(const CaveTalk_Handle_t *const handle, const CaveTalk_MetersPerSecond_t speed, const CaveTalk_RadiansPerSecond_t turn_rate)
{
    CaveTalk_Error_t error = CAVE_TALK_ERROR_NULL;

    if ((NULL == handle) || (NULL == handle->buffer) || (NULL == handle->link_handle.send))
    {
    }
    else
    {
        pb_ostream_t       ostream          = pb_ostream_from_buffer(handle->buffer, handle->buffer_size);
        cave_talk_Movement movement_message = cave_talk_Movement_init_zero;

        movement_message.speed_meters_per_second      = speed;
        movement_message.turn_rate_radians_per_second = turn_rate;

        if (!pb_encode(&ostream, cave_talk_Movement_fields, &movement_message))
        {
            error = CAVE_TALK_ERROR_SIZE;
        }
        else
        {
            error = CaveTalk_Speak(&handle->link_handle, (CaveTalk_Id_t)cave_talk_Id_ID_MOVEMENT, handle->buffer, ostream.bytes_written);
        }
    }

    return error;
}

CaveTalk_Error_t CaveTalk_SpeakCameraMovement(const CaveTalk_Handle_t *const handle, const CaveTalk_Radian_t pan, const CaveTalk_Radian_t tilt)
{
    CaveTalk_Error_t error = CAVE_TALK_ERROR_NULL;

    if ((NULL == handle) || (NULL == handle->buffer) || (NULL == handle->link_handle.send))
    {
    }
    else
    {
        pb_ostream_t             ostream                 = pb_ostream_from_buffer(handle->buffer, handle->buffer_size);
        cave_talk_CameraMovement camera_movement_message = cave_talk_CameraMovement_init_zero;

        camera_movement_message.pan_angle_radians  = pan;
        camera_movement_message.tilt_angle_radians = tilt;

        if (!pb_encode(&ostream, cave_talk_CameraMovement_fields, &camera_movement_message))
        {
            error = CAVE_TALK_ERROR_SIZE;
        }
        else
        {
            error = CaveTalk_Speak(&handle->link_handle, (CaveTalk_Id_t)cave_talk_Id_ID_CAMERA_MOVEMENT, handle->buffer, ostream.bytes_written);
        }
    }

    return error;
}

CaveTalk_Error_t CaveTalk_SpeakLights(const CaveTalk_Handle_t *const handle, const bool headlights)
{
    CaveTalk_Error_t error = CAVE_TALK_ERROR_NULL;

    if ((NULL == handle) || (NULL == handle->buffer) || (NULL == handle->link_handle.send))
    {
    }
    else
    {
        pb_ostream_t     ostream        = pb_ostream_from_buffer(handle->buffer, handle->buffer_size);
        cave_talk_Lights lights_message = cave_talk_Lights_init_zero;

        lights_message.headlights = headlights;

        if (!pb_encode(&ostream, cave_talk_Lights_fields, &lights_message))
        {
            error = CAVE_TALK_ERROR_SIZE;
        }
        else
        {
            error = CaveTalk_Speak(&handle->link_handle, (CaveTalk_Id_t)cave_talk_Id_ID_LIGHTS, handle->buffer, ostream.bytes_written);
        }
    }

    return error;
}

CaveTalk_Error_t CaveTalk_SpeakMode(const CaveTalk_Handle_t *const handle, const bool manual)
{
    CaveTalk_Error_t error = CAVE_TALK_ERROR_NULL;

    if ((NULL == handle) || (NULL == handle->buffer) || (NULL == handle->link_handle.send))
    {
    }
    else
    {
        pb_ostream_t   ostream      = pb_ostream_from_buffer(handle->buffer, handle->buffer_size);
        cave_talk_Mode mode_message = cave_talk_Mode_init_zero;

        mode_message.manual = manual;

        if (!pb_encode(&ostream, cave_talk_Mode_fields, &mode_message))
        {
            error = CAVE_TALK_ERROR_SIZE;
        }
        else
        {
            error = CaveTalk_Speak(&handle->link_handle, (CaveTalk_Id_t)cave_talk_Id_ID_MODE, handle->buffer, ostream.bytes_written);
        }
    }

    return error;
}


CaveTalk_Error_t CaveTalk_SpeakConfigServoWheels(const CaveTalk_Handle_t *const handle, const cave_talk_Servo servo_wheel_0, const cave_talk_Servo servo_wheel_1, const cave_talk_Servo servo_wheel_2, const cave_talk_Servo servo_wheel_3)
{
    CaveTalk_Error_t error = CAVE_TALK_ERROR_NULL;

    if ((NULL == handle) || (NULL == handle->buffer) || (NULL == handle->link_handle.send))
    {
    }
    else
    {
        pb_ostream_t                ostream                     = pb_ostream_from_buffer(handle->buffer, handle->buffer_size);
        cave_talk_ConfigServoWheels config_servo_wheels_message = cave_talk_ConfigServoWheels_init_zero;

        config_servo_wheels_message.servo_wheel_0     = servo_wheel_0;
        config_servo_wheels_message.has_servo_wheel_0 = true;

        config_servo_wheels_message.servo_wheel_1     = servo_wheel_1;
        config_servo_wheels_message.has_servo_wheel_1 = true;

        config_servo_wheels_message.servo_wheel_2     = servo_wheel_2;
        config_servo_wheels_message.has_servo_wheel_2 = true;

        config_servo_wheels_message.servo_wheel_3     = servo_wheel_3;
        config_servo_wheels_message.has_servo_wheel_3 = true;

        if (!pb_encode(&ostream, cave_talk_ConfigServoWheels_fields, &config_servo_wheels_message))
        {
            error = CAVE_TALK_ERROR_SIZE;
        }
        else
        {
            error = CaveTalk_Speak(&handle->link_handle, (CaveTalk_Id_t)cave_talk_Id_ID_CONFIG_SERVO_WHEELS, handle->buffer, ostream.bytes_written);
        }


    }

    return error;

}

CaveTalk_Error_t CaveTalk_SpeakConfigServoCams(const CaveTalk_Handle_t *const handle, const cave_talk_Servo servo_cam_pan, const cave_talk_Servo servo_cam_tilt)
{
    CaveTalk_Error_t error = CAVE_TALK_ERROR_NULL;

    if ((NULL == handle) || (NULL == handle->buffer) || (NULL == handle->link_handle.send))
    {
    }
    else
    {
        pb_ostream_t              ostream                   = pb_ostream_from_buffer(handle->buffer, handle->buffer_size);
        cave_talk_ConfigServoCams config_servo_cams_message = cave_talk_ConfigServoCams_init_zero;

        config_servo_cams_message.servo_cam_pan     = servo_cam_pan;
        config_servo_cams_message.has_servo_cam_pan = true;

        config_servo_cams_message.servo_cam_tilt     = servo_cam_tilt;
        config_servo_cams_message.has_servo_cam_tilt = true;

        if (!pb_encode(&ostream, cave_talk_ConfigServoCams_fields, &config_servo_cams_message))
        {
            error = CAVE_TALK_ERROR_SIZE;
        }
        else
        {
            error = CaveTalk_Speak(&handle->link_handle, (CaveTalk_Id_t)cave_talk_Id_ID_CONFIG_SERVO_CAMS, handle->buffer, ostream.bytes_written);
        }


    }

    return error;

}

CaveTalk_Error_t CaveTalk_SpeakConfigMotors(const CaveTalk_Handle_t *const handle, const cave_talk_Motor motor_wheel_0, const cave_talk_Motor motor_wheel_1, const cave_talk_Motor motor_wheel_2, const cave_talk_Motor motor_wheel_3)
{
    CaveTalk_Error_t error = CAVE_TALK_ERROR_NULL;

    if ((NULL == handle) || (NULL == handle->buffer) || (NULL == handle->link_handle.send))
    {
    }
    else
    {
        pb_ostream_t          ostream               = pb_ostream_from_buffer(handle->buffer, handle->buffer_size);
        cave_talk_ConfigMotor config_motors_message = cave_talk_ConfigMotor_init_zero;

        config_motors_message.motor_wheel_0     = motor_wheel_0;
        config_motors_message.has_motor_wheel_0 = true;


        config_motors_message.motor_wheel_1     = motor_wheel_1;
        config_motors_message.has_motor_wheel_1 = true;

        config_motors_message.motor_wheel_2     = motor_wheel_2;
        config_motors_message.has_motor_wheel_2 = true;

        config_motors_message.motor_wheel_3     = motor_wheel_3;
        config_motors_message.has_motor_wheel_3 = true;


        if (!pb_encode(&ostream, cave_talk_ConfigMotor_fields, &config_motors_message))
        {
            error = CAVE_TALK_ERROR_SIZE;
        }
        else
        {
            error = CaveTalk_Speak(&handle->link_handle, (CaveTalk_Id_t)cave_talk_Id_ID_CONFIG_MOTOR, handle->buffer, ostream.bytes_written);
        }


    }

    return error;

}


static CaveTalk_Error_t CaveTalk_HandleOogaBooga(const CaveTalk_Handle_t *const handle, const CaveTalk_Length_t length)

{
    CaveTalk_Error_t error = CAVE_TALK_ERROR_NONE;

    if ((NULL == handle) || (NULL == handle->buffer))
    {
        error = CAVE_TALK_ERROR_NULL;
    }
    else
    {
        pb_istream_t        istream            = pb_istream_from_buffer(handle->buffer, length);
        cave_talk_OogaBooga ooga_booga_message = cave_talk_OogaBooga_init_zero;

        if (!pb_decode(&istream, cave_talk_OogaBooga_fields, &ooga_booga_message))
        {
            error = CAVE_TALK_ERROR_PARSE;
        }
        else if (NULL != handle->listen_callbacks.hear_ooga_booga)
        {
            handle->listen_callbacks.hear_ooga_booga(ooga_booga_message.ooga_booga);
        }
    }

    return error;
}

static CaveTalk_Error_t CaveTalk_HandleMovement(const CaveTalk_Handle_t *const handle, const CaveTalk_Length_t length)
{
    CaveTalk_Error_t error = CAVE_TALK_ERROR_NONE;

    if ((NULL == handle) || (NULL == handle->buffer))
    {
        error = CAVE_TALK_ERROR_NULL;
    }
    else
    {
        pb_istream_t       istream          = pb_istream_from_buffer(handle->buffer, length);
        cave_talk_Movement movement_message = cave_talk_Movement_init_zero;

        if (!pb_decode(&istream, cave_talk_Movement_fields, &movement_message))
        {
            error = CAVE_TALK_ERROR_PARSE;
        }
        else if (NULL != handle->listen_callbacks.hear_movement)
        {
            handle->listen_callbacks.hear_movement(movement_message.speed_meters_per_second, movement_message.turn_rate_radians_per_second);
        }
    }

    return error;
}

static CaveTalk_Error_t CaveTalk_HandleCameraMovement(const CaveTalk_Handle_t *const handle, const CaveTalk_Length_t length)
{
    CaveTalk_Error_t error = CAVE_TALK_ERROR_NONE;

    if ((NULL == handle) || (NULL == handle->buffer))
    {
        error = CAVE_TALK_ERROR_NULL;
    }
    else
    {
        pb_istream_t             istream                 = pb_istream_from_buffer(handle->buffer, length);
        cave_talk_CameraMovement camera_movement_message = cave_talk_CameraMovement_init_zero;

        if (!pb_decode(&istream, cave_talk_CameraMovement_fields, &camera_movement_message))
        {
            error = CAVE_TALK_ERROR_PARSE;
        }
        else if (NULL != handle->listen_callbacks.hear_camera_movement)
        {
            handle->listen_callbacks.hear_camera_movement(camera_movement_message.pan_angle_radians, camera_movement_message.tilt_angle_radians);
        }
    }

    return error;
}

static CaveTalk_Error_t CaveTalk_HandleLights(const CaveTalk_Handle_t *const handle, const CaveTalk_Length_t length)
{
    CaveTalk_Error_t error = CAVE_TALK_ERROR_NONE;

    if ((NULL == handle) || (NULL == handle->buffer))
    {
        error = CAVE_TALK_ERROR_NULL;
    }
    else
    {
        pb_istream_t     istream        = pb_istream_from_buffer(handle->buffer, length);
        cave_talk_Lights lights_message = cave_talk_Lights_init_zero;

        if (!pb_decode(&istream, cave_talk_Lights_fields, &lights_message))
        {
            error = CAVE_TALK_ERROR_PARSE;
        }
        else if (NULL != handle->listen_callbacks.hear_lights)
        {
            handle->listen_callbacks.hear_lights(lights_message.headlights);
        }
    }

    return error;
}

static CaveTalk_Error_t CaveTalk_HandleMode(const CaveTalk_Handle_t *const handle, const CaveTalk_Length_t length)
{
    CaveTalk_Error_t error = CAVE_TALK_ERROR_NONE;

    if ((NULL == handle) || (NULL == handle->buffer))
    {
        error = CAVE_TALK_ERROR_NULL;
    }
    else
    {
        pb_istream_t   istream      = pb_istream_from_buffer(handle->buffer, length);
        cave_talk_Mode mode_message = cave_talk_Mode_init_zero;

        if (!pb_decode(&istream, cave_talk_Mode_fields, &mode_message))
        {
            error = CAVE_TALK_ERROR_PARSE;
        }
        else if (NULL != handle->listen_callbacks.hear_mode)
        {
            handle->listen_callbacks.hear_mode(mode_message.manual);
        }
    }

    return error;
}

static CaveTalk_Error_t CaveTalk_HandleConfigServoWheels(const CaveTalk_Handle_t *const handle, const CaveTalk_Length_t length)
{
    CaveTalk_Error_t error = CAVE_TALK_ERROR_NONE;

    if ((NULL == handle) || (NULL == handle->buffer))
    {
        error = CAVE_TALK_ERROR_NULL;
    }
    else
    {
        pb_istream_t                istream                     = pb_istream_from_buffer(handle->buffer, length);
        cave_talk_ConfigServoWheels config_servo_wheels_message = cave_talk_ConfigServoWheels_init_zero;

        if (!pb_decode(&istream, cave_talk_ConfigServoWheels_fields, &config_servo_wheels_message))
        {
            error = CAVE_TALK_ERROR_PARSE;
        }
        else if (NULL != handle->listen_callbacks.hear_config_servo_wheels)
        {
            handle->listen_callbacks.hear_config_servo_wheels(config_servo_wheels_message.servo_wheel_0, config_servo_wheels_message.servo_wheel_1, config_servo_wheels_message.servo_wheel_2, config_servo_wheels_message.servo_wheel_3);
        }
    }

    return error;

}

static CaveTalk_Error_t CaveTalk_HandleConfigServoCams(const CaveTalk_Handle_t *const handle, const CaveTalk_Length_t length)
{
    CaveTalk_Error_t error = CAVE_TALK_ERROR_NONE;

    if ((NULL == handle) || (NULL == handle->buffer))
    {
        error = CAVE_TALK_ERROR_NULL;
    }
    else
    {
        pb_istream_t              istream                   = pb_istream_from_buffer(handle->buffer, length);
        cave_talk_ConfigServoCams config_servo_cams_message = cave_talk_ConfigServoCams_init_zero;

        if (!pb_decode(&istream, cave_talk_ConfigServoCams_fields, &config_servo_cams_message))
        {
            error = CAVE_TALK_ERROR_PARSE;
        }
        else if (NULL != handle->listen_callbacks.hear_config_servo_cams)
        {
            handle->listen_callbacks.hear_config_servo_cams(config_servo_cams_message.servo_cam_pan, config_servo_cams_message.servo_cam_tilt);
        }
    }

    return error;

}

static CaveTalk_Error_t CaveTalk_HandleConfigMotor(const CaveTalk_Handle_t *const handle, const CaveTalk_Length_t length)
{
    CaveTalk_Error_t error = CAVE_TALK_ERROR_NONE;

    if ((NULL == handle) || (NULL == handle->buffer))
    {
        error = CAVE_TALK_ERROR_NULL;
    }
    else
    {
        pb_istream_t          istream              = pb_istream_from_buffer(handle->buffer, length);
        cave_talk_ConfigMotor config_motor_message = cave_talk_ConfigMotor_init_zero;

        if (!pb_decode(&istream, cave_talk_ConfigMotor_fields, &config_motor_message))
        {
            error = CAVE_TALK_ERROR_PARSE;
        }
        else if (NULL != handle->listen_callbacks.hear_config_motors)
        {
            handle->listen_callbacks.hear_config_motors(config_motor_message.motor_wheel_0, config_motor_message.motor_wheel_1, config_motor_message.motor_wheel_2, config_motor_message.motor_wheel_3);
        }
    }

    return error;

}
