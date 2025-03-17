#include <cstddef>

#include <gtest/gtest.h>
#include <gmock/gmock.h>

#include "ooga_booga.pb.h"
#include "config_servo.pb.h"
#include "config_motor.pb.h"

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

void HearOdometry(const cave_talk_Imu *const imu, const cave_talk_Encoder *const encoder_wheel_0, const cave_talk_Encoder *const encoder_wheel_1, const cave_talk_Encoder *const encoder_wheel_2, const cave_talk_Encoder *const encoder_wheel_3)
{
    std::cout << "Just do a breakpoint here" << std::endl;
    return;
}

void HearLog(const char *const log)
{
    std::cout << std::string(log) << std::endl;
    return;
}

void HearConfigServoWheels(const cave_talk_Servo *const servo_wheel_0, const cave_talk_Servo *const servo_wheel_1, const cave_talk_Servo *const servo_wheel_2, const cave_talk_Servo *const servo_wheel_3)
{
    std::cout << "Just do a breakpoint here" << std::endl;
    return;
}

void HearConfigServoCams(const cave_talk_Servo *const servo_cam_pan, const cave_talk_Servo *const servo_cam_tilt)
{
    std::cout << "Just do a breakpoint here" << std::endl;
    return;
}

void HearConfigServoMotors(const cave_talk_Motor *const motor_wheel_0, const cave_talk_Motor *const motor_wheel_1, const cave_talk_Motor *const motor_wheel_2, const cave_talk_Motor *const motor_wheel_3)
{
    std::cout << "Just do a breakpoint here" << std::endl;
    return;
}

void HearConfigEncoder(const cave_talk_ConfigEncoder *const encoder_wheel_0, const cave_talk_ConfigEncoder *const encoder_wheel_1, const cave_talk_ConfigEncoder *const encoder_wheel_2, const cave_talk_ConfigEncoder *const encoder_wheel_3)
{
    std::cout << "Just do a breakpoint here" << std::endl;
    return;
}



const CaveTalk_ListenCallbacks_t kCaveTalk_ListenCallbacksInterface = {
    .hear_ooga_booga      = HearOogaBooga,
    .hear_movement        = HearMovement,
    .hear_camera_movement = HearCameraMovement,
    .hear_lights          = HearLights,
    .hear_mode            = HearMode,
    .hear_odometry = HearOdometry,
    .hear_log = HearLog,
    .hear_config_servo_wheels = HearConfigServoWheels,
    .hear_config_servo_cams = HearConfigServoCams,
    .hear_config_motors = HearConfigServoMotors,
    .hear_config_encoders = HearConfigEncoder,
};

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

TEST(CaveTalkCTests, SpeakListenOdometry)
{

    ring_buffer.Clear();

    cave_talk_Imu imu = cave_talk_Imu();
    imu.has_accel = true;
    imu.accel.x_meters_per_second_squared = .00004;
    imu.accel.y_meters_per_second_squared = .03004;
    imu.accel.z_meters_per_second_squared = 152352.2038492;
    
    imu.has_gyro = true;
    imu.gyro.pitch_radians_per_second = 17029348.57032894;
    imu.gyro.roll_radians_per_second = 123.00000000000001;
    imu.gyro.yaw_radians_per_second = 82482.1111111111111;

    cave_talk_Encoder encoder_test = cave_talk_Encoder();
    encoder_test.rate_radians_per_second = 6.1412341231;
    encoder_test.total_pulses = 999921;


    ASSERT_EQ(CAVE_TALK_ERROR_NONE, CaveTalk_SpeakOdometry(&CaveTalk_Handle, &imu, &encoder_test, &encoder_test, &encoder_test, &encoder_test));
    //You would have an EXPECT_CALL here for HearOdometry but there is no operator== for Imu & Encoder
    // enter debug mode and you can see that it is called with the correct params
    ASSERT_EQ(CAVE_TALK_ERROR_NONE, CaveTalk_Hear(&CaveTalk_Handle));

}

TEST(CaveTalkCTests, SpeakListenLog)
{
    ring_buffer.Clear();

    char hw[] = "Hello World! 12401928347";

    ASSERT_EQ(CAVE_TALK_ERROR_NONE, CaveTalk_SpeakLog(&CaveTalk_Handle, hw));
    //Expect Call
    ASSERT_EQ(CAVE_TALK_ERROR_NONE, CaveTalk_Hear(&CaveTalk_Handle));



}

TEST(CaveTalkCTests, SpeakListenConfigServoWheels)
{

    ring_buffer.Clear();

    cave_talk_Servo servo_test_zero = cave_talk_Servo();
    (servo_test_zero).min_angle_radian = (0.2);
    (servo_test_zero).max_angle_radian = (180.5);
    (servo_test_zero).center_angle_radian = (94.3);
    (servo_test_zero).min_duty_cycle_microseconds = (540);
    (servo_test_zero).max_duty_cycle_microseconds = (2560);
    (servo_test_zero).center_duty_cycle_microseconds = (1576);


    ASSERT_EQ(CAVE_TALK_ERROR_NONE, CaveTalk_SpeakConfigServoWheels(&CaveTalk_Handle, &servo_test_zero, &servo_test_zero, &servo_test_zero, &servo_test_zero));
    //You would have an EXPECT_CALL here for HearConfigServoWheels but there is no operator== for class Servo
    // enter debug mode and you can see that it is called with the correct params
    ASSERT_EQ(CAVE_TALK_ERROR_NONE, CaveTalk_Hear(&CaveTalk_Handle));

}

TEST(CaveTalkCTests, SpeakListenConfigServoCams)
{

    ring_buffer.Clear();

    cave_talk_Servo servo_test_zero = cave_talk_Servo();
    (servo_test_zero).min_angle_radian = (0.2);
    (servo_test_zero).max_angle_radian = (180.5);
    (servo_test_zero).center_angle_radian = (94.3);
    (servo_test_zero).min_duty_cycle_microseconds = (540);
    (servo_test_zero).max_duty_cycle_microseconds = (2560);
    (servo_test_zero).center_duty_cycle_microseconds = (1576);

    ASSERT_EQ(CAVE_TALK_ERROR_NONE, CaveTalk_SpeakConfigServoCams(&CaveTalk_Handle, &servo_test_zero, &servo_test_zero));
    //You would have an EXPECT_CALL here for HearConfigServoCams but there is no operator== for class Servo
    // enter debug mode and you can see that it is called with the correct params
    ASSERT_EQ(CAVE_TALK_ERROR_NONE, CaveTalk_Hear(&CaveTalk_Handle));

}

TEST(CaveTalkCTests, SpeakListenConfigMotors)
{

    ring_buffer.Clear();

    cave_talk_Motor motor_test_zero = cave_talk_Motor();
    (motor_test_zero).pwm_carrier_freq_hz = (2500);
    (motor_test_zero).min_speed_loaded_meters_per_second = (0.3);
    (motor_test_zero).max_speed_loaded_meters_per_second = (2.34);
    (motor_test_zero).min_duty_cycle_percentage = (540);
    (motor_test_zero).max_duty_cycle_percentage = (2560);

    ASSERT_EQ(CAVE_TALK_ERROR_NONE, CaveTalk_SpeakConfigMotors(&CaveTalk_Handle, &motor_test_zero, &motor_test_zero, &motor_test_zero, &motor_test_zero));
    //You would have an EXPECT_CALL here for HearConfigMotors but there is no operator== for class Servo
    // enter debug mode and you can see that it is called with the correct params
    ASSERT_EQ(CAVE_TALK_ERROR_NONE, CaveTalk_Hear(&CaveTalk_Handle));

}

TEST(CaveTalkCTests, SpeakListenConfigEncoder)
{

    ring_buffer.Clear();

    cave_talk_ConfigEncoder config_encoder_test = cave_talk_ConfigEncoder();
    config_encoder_test.smoothing_factor = (.005);
    config_encoder_test.mode = (cave_talk_EncoderMode::cave_talk_EncoderMode_BSP_ENCODER_USER_MODE_PULSES_PER_ROTATON);
    config_encoder_test.pulses_per_period = (6.2304);
    config_encoder_test.radians_per_pulse = (3.000000001);

    ASSERT_EQ(CAVE_TALK_ERROR_NONE, CaveTalk_SpeakConfigEncoders(&CaveTalk_Handle, &config_encoder_test, &config_encoder_test, &config_encoder_test, &config_encoder_test));
    //You would have an EXPECT_CALL here for HearConfigMotors but there is no operator== for class Servo
    // enter debug mode and you can see that it is called with the correct params
    ASSERT_EQ(CAVE_TALK_ERROR_NONE, CaveTalk_Hear(&CaveTalk_Handle));

}