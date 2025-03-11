#include <cstddef>
#include <functional>
#include <string>

#include <gtest/gtest.h>
#include <gmock/gmock.h>

#include "ooga_booga.pb.h"
#include "config_servo.pb.h"
#include "config_motor.pb.h"
#include "odometry.pb.h"

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
        MOCK_METHOD(void, HearLog, (const CaveTalk_Message_t), (override));
        MOCK_METHOD(void, HearOdometry, ((const cave_talk::Imu&), (const cave_talk::Encoder&), (const cave_talk::Encoder&), (const cave_talk::Encoder&), (const cave_talk::Encoder&)), (override));
        MOCK_METHOD(void, HearConfigServoWheels, ((const cave_talk::Servo&),(const cave_talk::Servo&),(const cave_talk::Servo&),(const cave_talk::Servo&)), (override));
        MOCK_METHOD(void, HearConfigServoCams, ((const cave_talk::Servo&),(const cave_talk::Servo&)), (override));
        MOCK_METHOD(void, HearConfigMotor, ((const cave_talk::Motor&),(const cave_talk::Motor&),(const cave_talk::Motor&),(const cave_talk::Motor&)), (override));

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


TEST(CaveTalkCppTests, SpeakListenOogaBooga){

    std::shared_ptr<MockListenerCallbacks> mock_listen_callbacks = std::make_shared<MockListenerCallbacks>();
    cave_talk::Talker roverMouth(Send);
    cave_talk::Listener roverEars(Receive, mock_listen_callbacks);

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

    std::shared_ptr<MockListenerCallbacks> mock_listen_callbacks = std::make_shared<MockListenerCallbacks>();
    cave_talk::Talker roverMouth(Send);
    cave_talk::Listener roverEars(Receive, mock_listen_callbacks);

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

    std::shared_ptr<MockListenerCallbacks> mock_listen_callbacks = std::make_shared<MockListenerCallbacks>();
    cave_talk::Talker roverMouth(Send);
    cave_talk::Listener roverEars(Receive, mock_listen_callbacks);

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

    std::shared_ptr<MockListenerCallbacks> mock_listen_callbacks = std::make_shared<MockListenerCallbacks>();
    cave_talk::Talker roverMouth(Send);
    cave_talk::Listener roverEars(Receive, mock_listen_callbacks);

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

    std::shared_ptr<MockListenerCallbacks> mock_listen_callbacks = std::make_shared<MockListenerCallbacks>();
    cave_talk::Talker roverMouth(Send);
    cave_talk::Listener roverEars(Receive, mock_listen_callbacks);

    ring_buffer.Clear();

    ASSERT_EQ(CAVE_TALK_ERROR_NONE, roverMouth.SpeakMode(true));
    EXPECT_CALL(*mock_listen_callbacks.get(), HearMode(true)).Times(1);
    ASSERT_EQ(CAVE_TALK_ERROR_NONE, roverEars.Listen());

    ring_buffer.Clear();

    ASSERT_EQ(CAVE_TALK_ERROR_NONE, roverMouth.SpeakMode(false));
    EXPECT_CALL(*mock_listen_callbacks.get(), HearMode(false)).Times(1);
    ASSERT_EQ(CAVE_TALK_ERROR_NONE, roverEars.Listen());
    
}



TEST(CaveTalkCppTests, SpeakListenOdometry)
{
    std::shared_ptr<MockListenerCallbacks> mock_listen_callbacks = std::make_shared<MockListenerCallbacks>();
    cave_talk::Talker roverMouth(Send);
    cave_talk::Listener roverEars(Receive, mock_listen_callbacks);

    ring_buffer.Clear();

    cave_talk::Accelerometer accel;
    accel.set_x_meters_per_second_squared(.0043);
    accel.set_y_meters_per_second_squared(.142);
    accel.set_z_meters_per_second_squared(5.2983);
    
    cave_talk::Gyroscope gyro;
    gyro.set_pitch_radians_per_second(2.813);
    gyro.set_roll_radians_per_second(7.31342453);
    gyro.set_yaw_radians_per_second(9.34232352);

    cave_talk::Imu IMU;
    IMU.mutable_accel()->CopyFrom(accel);
    IMU.mutable_gyro()->CopyFrom(gyro);


    cave_talk::Encoder encoder_0;
    encoder_0.set_rate_radians_per_second(3.141592652);
    encoder_0.set_total_pulses(100000);

    ASSERT_EQ(CAVE_TALK_ERROR_NONE, roverMouth.SpeakOdometry(IMU, encoder_0, encoder_0, encoder_0, encoder_0));
    //You would have an EXPECT_CALL here for HearConfigServoWheels but there is no operator== for class IMU or Encoder
    // enter debug mode and you can see that it is called with the correct params
    ASSERT_EQ(CAVE_TALK_ERROR_NONE, roverEars.Listen());
}



TEST(CaveTalkCppTests, SpeakListenConfigServoWheels)
{

    std::shared_ptr<MockListenerCallbacks> mock_listen_callbacks = std::make_shared<MockListenerCallbacks>();
    cave_talk::Talker roverMouth(Send);
    cave_talk::Listener roverEars(Receive, mock_listen_callbacks);

    ring_buffer.Clear();

    cave_talk::Servo servo_test_zero;
    servo_test_zero.set_min_angle_radian(0.2);
    servo_test_zero.set_max_angle_radian(180.5);
    servo_test_zero.set_center_angle_radian(94.3);
    servo_test_zero.set_min_duty_cycle_microseconds(540);
    servo_test_zero.set_max_duty_cycle_microseconds(2560);
    servo_test_zero.set_center_duty_cycle_microseconds(1576);

    ASSERT_EQ(CAVE_TALK_ERROR_NONE, roverMouth.SpeakConfigServoWheels(servo_test_zero, servo_test_zero, servo_test_zero, servo_test_zero));
    //You would have an EXPECT_CALL here for HearConfigServoWheels but there is no operator== for class Servo
    // enter debug mode and you can see that it is called with the correct params
    ASSERT_EQ(CAVE_TALK_ERROR_NONE, roverEars.Listen());

}

TEST(CaveTalkCppTests, SpeakListenConfigServoCams)
{

    std::shared_ptr<MockListenerCallbacks> mock_listen_callbacks = std::make_shared<MockListenerCallbacks>();
    cave_talk::Talker roverMouth(Send);
    cave_talk::Listener roverEars(Receive, mock_listen_callbacks);

    ring_buffer.Clear();

    cave_talk::Servo servo_test_zero;
    servo_test_zero.set_min_angle_radian(0.2);
    servo_test_zero.set_max_angle_radian(180.5);
    servo_test_zero.set_center_angle_radian(94.3);
    servo_test_zero.set_min_duty_cycle_microseconds(540);
    servo_test_zero.set_max_duty_cycle_microseconds(2560);
    servo_test_zero.set_center_duty_cycle_microseconds(1576);



    ASSERT_EQ(CAVE_TALK_ERROR_NONE, roverMouth.SpeakConfigServoCams(servo_test_zero, servo_test_zero));
    //You would have an EXPECT_CALL here for HearConfigServoCams but there is no operator== for class Servo
    // enter debug mode and you can see that it is called with the correct params
    ASSERT_EQ(CAVE_TALK_ERROR_NONE, roverEars.Listen());

}

TEST(CaveTalkCppTests, SpeakListenConfigMotors)
{

    std::shared_ptr<MockListenerCallbacks> mock_listen_callbacks = std::make_shared<MockListenerCallbacks>();
    cave_talk::Talker roverMouth(Send);
    cave_talk::Listener roverEars(Receive, mock_listen_callbacks);

    ring_buffer.Clear();

    cave_talk::Motor motor_test_zero;
    motor_test_zero.set_pwm_carrier_freq_hz(2500);
    motor_test_zero.set_min_speed_loaded_meters_per_second(0.3);
    motor_test_zero.set_max_speed_loaded_meters_per_second(2.34);
    motor_test_zero.set_min_duty_cycle_percentage(540);
    motor_test_zero.set_max_duty_cycle_percentage(2560);

    ASSERT_EQ(CAVE_TALK_ERROR_NONE, roverMouth.SpeakConfigMotor(motor_test_zero, motor_test_zero, motor_test_zero, motor_test_zero));
    //You would have an EXPECT_CALL here for HearConfigMotors but there is no operator== for class Motor
    // enter debug mode and you can see that it is called with the correct params
    ASSERT_EQ(CAVE_TALK_ERROR_NONE, roverEars.Listen());


}