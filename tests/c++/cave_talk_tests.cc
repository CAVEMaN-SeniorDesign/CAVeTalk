#include <cstddef>
#include <functional>
#include <string>

#include <gtest/gtest.h>
#include <gmock/gmock.h>

#include "ooga_booga.pb.h"
#include "config_encoder.pb.h"
#include "config_log.pb.h"
#include "config_motor.pb.h"
#include "config_pid.pb.h"
#include "config_servo.pb.h"
#include "odometry.pb.h"

#include "cave_talk.h"
#include "cave_talk_link.h"
#include "cave_talk_types.h"
#include "ring_buffer.h"

static const std::size_t kMaxMessageLength = 255U;
static RingBuffer<uint8_t, kMaxMessageLength> ring_buffer;
cave_talk::Imu imu_odometry_saved;
cave_talk::Encoder encoder_odometry_saved_0;
cave_talk::Encoder encoder_odometry_saved_1;
cave_talk::Encoder encoder_odometry_saved_2;
cave_talk::Encoder encoder_odometry_saved_3;
cave_talk::Servo servo_configservowheels_saved_0;
cave_talk::Servo servo_configservowheels_saved_1;
cave_talk::Servo servo_configservowheels_saved_2;
cave_talk::Servo servo_configservowheels_saved_3;
cave_talk::Servo servo_configservocams_saved_pan;
cave_talk::Servo servo_configservocams_saved_tilt;
cave_talk::Motor motor_configmotor_saved_0;
cave_talk::Motor motor_configmotor_saved_1;
cave_talk::Motor motor_configmotor_saved_2;
cave_talk::Motor motor_configmotor_saved_3;
cave_talk::ConfigEncoder configencoder_configencoder_saved_0;
cave_talk::ConfigEncoder configencoder_configencoder_saved_1;
cave_talk::ConfigEncoder configencoder_configencoder_saved_2;
cave_talk::ConfigEncoder configencoder_configencoder_saved_3;
cave_talk::PID pid_wsc_saved_0;
cave_talk::PID pid_wsc_saved_1;
cave_talk::PID pid_wsc_saved_2;
cave_talk::PID pid_wsc_saved_3;
cave_talk::PID pid_sc_saved_trp;

void TestIMUObject(const cave_talk::Imu &a, const cave_talk::Imu &b)
{
    ASSERT_EQ(a.accel().x_meters_per_second_squared(), b.accel().x_meters_per_second_squared());
    ASSERT_EQ(a.accel().y_meters_per_second_squared(), b.accel().y_meters_per_second_squared());
    ASSERT_EQ(a.accel().z_meters_per_second_squared(), b.accel().z_meters_per_second_squared());
    ASSERT_EQ(a.gyro().roll_radians_per_second(), b.gyro().roll_radians_per_second());
    ASSERT_EQ(a.gyro().pitch_radians_per_second(), b.gyro().pitch_radians_per_second());
    ASSERT_EQ(a.gyro().yaw_radians_per_second(), b.gyro().yaw_radians_per_second());
    ASSERT_EQ(a.quat().w(), b.quat().w());
    ASSERT_EQ(a.quat().x(), b.quat().x());
    ASSERT_EQ(a.quat().y(), b.quat().y());
    ASSERT_EQ(a.quat().z(), b.quat().z());
}

void TestEncoderObject(const cave_talk::Encoder &a, const cave_talk::Encoder &b)
{
    ASSERT_EQ(a.total_pulses(), b.total_pulses());
    ASSERT_EQ(a.rate_radians_per_second(), b.rate_radians_per_second());
}

void TestServoObject(const cave_talk::Servo &a, const cave_talk::Servo &b)
{
    ASSERT_EQ(a.min_angle_radian(), b.min_angle_radian());
    ASSERT_EQ(a.max_angle_radian(), b.max_angle_radian());
    ASSERT_EQ(a.center_angle_radian(), b.center_angle_radian());
    ASSERT_EQ(a.min_duty_cycle_percentage(), b.min_duty_cycle_percentage());
    ASSERT_EQ(a.max_duty_cycle_percentage(), b.max_duty_cycle_percentage());
    ASSERT_EQ(a.center_duty_cycle_percentage(), b.center_duty_cycle_percentage());
}

void TestMotorObject(const cave_talk::Motor &a, const cave_talk::Motor &b)
{
    ASSERT_EQ(a.pwm_carrier_freq_hz(), b.pwm_carrier_freq_hz());
    ASSERT_EQ(a.min_duty_cycle_percentage(), b.min_duty_cycle_percentage());
    ASSERT_EQ(a.max_duty_cycle_percentage(), b.max_duty_cycle_percentage());
    ASSERT_EQ(a.min_speed_loaded_meters_per_second(), b.min_speed_loaded_meters_per_second());
    ASSERT_EQ(a.max_speed_loaded_meters_per_second(), b.max_speed_loaded_meters_per_second());
}

void TestConfigEncoderObject(const cave_talk::ConfigEncoder &a, const cave_talk::ConfigEncoder &b)
{
    ASSERT_EQ(a.smoothing_factor(), b.smoothing_factor());
    ASSERT_EQ(a.radians_per_pulse(), b.radians_per_pulse());
    ASSERT_EQ(a.pulses_per_period(), b.pulses_per_period());
    ASSERT_EQ(a.mode(), b.mode());
}

void TestPIDObject(const cave_talk::PID &a, const cave_talk::PID &b)
{
    ASSERT_EQ(a.kp(), b.kp());
    ASSERT_EQ(a.ki(), b.ki());
    ASSERT_EQ(a.kd(), b.kd());
}

class MockListenerCallbacks : public cave_talk::ListenerCallbacks
{
public:
    MOCK_METHOD(void, HearOogaBooga, (const cave_talk::Say), (override));
    MOCK_METHOD(void, HearMovement, ((const CaveTalk_MetersPerSecond_t), (const CaveTalk_RadiansPerSecond_t)), (override));
    MOCK_METHOD(void, HearCameraMovement, ((const CaveTalk_Radian_t), (const CaveTalk_Radian_t)), (override));
    MOCK_METHOD(void, HearLights, (const bool), (override));
    MOCK_METHOD(void, HearArm, (const bool), (override));
    MOCK_METHOD(void, HearLog, (const char *const), (override));

    void HearOdometry(const cave_talk::Imu &IMU, const cave_talk::Encoder &encoder_wheel_0, const cave_talk::Encoder &encoder_wheel_1, const cave_talk::Encoder &encoder_wheel_2, const cave_talk::Encoder &encoder_wheel_3) override
    {
        TestIMUObject(imu_odometry_saved, IMU);
        TestEncoderObject(encoder_odometry_saved_0, encoder_wheel_0);
        TestEncoderObject(encoder_odometry_saved_1, encoder_wheel_1);
        TestEncoderObject(encoder_odometry_saved_2, encoder_wheel_2);
        TestEncoderObject(encoder_odometry_saved_3, encoder_wheel_3);
    }

    void HearConfigServoWheels(const cave_talk::Servo &servo_wheel_0, const cave_talk::Servo &servo_wheel_1, const cave_talk::Servo &servo_wheel_2, const cave_talk::Servo &servo_wheel_3) override
    {
        TestServoObject(servo_configservowheels_saved_0, servo_wheel_0);
        TestServoObject(servo_configservowheels_saved_1, servo_wheel_1);
        TestServoObject(servo_configservowheels_saved_2, servo_wheel_2);
        TestServoObject(servo_configservowheels_saved_3, servo_wheel_3);
    }

    void HearConfigServoCams(const cave_talk::Servo &servo_cam_pan, const cave_talk::Servo &servo_cam_tilt)
    {
        TestServoObject(servo_configservocams_saved_pan, servo_cam_pan);
        TestServoObject(servo_configservocams_saved_tilt, servo_cam_tilt);
    }

    void HearConfigMotor(const cave_talk::Motor &motor_wheel_0, const cave_talk::Motor &motor_wheel_1, const cave_talk::Motor &motor_wheel_2, const cave_talk::Motor &motor_wheel_3)
    {
        TestMotorObject(motor_configmotor_saved_0, motor_wheel_0);
        TestMotorObject(motor_configmotor_saved_1, motor_wheel_1);
        TestMotorObject(motor_configmotor_saved_2, motor_wheel_2);
        TestMotorObject(motor_configmotor_saved_3, motor_wheel_3);
    }

    void HearConfigEncoder(const cave_talk::ConfigEncoder &encoder_wheel_0, const cave_talk::ConfigEncoder &encoder_wheel_1, const cave_talk::ConfigEncoder &encoder_wheel_2, const cave_talk::ConfigEncoder &encoder_wheel_3)
    {
        TestConfigEncoderObject(configencoder_configencoder_saved_0, encoder_wheel_0);
        TestConfigEncoderObject(configencoder_configencoder_saved_1, encoder_wheel_1);
        TestConfigEncoderObject(configencoder_configencoder_saved_2, encoder_wheel_2);
        TestConfigEncoderObject(configencoder_configencoder_saved_3, encoder_wheel_3);
    }

    MOCK_METHOD(void, HearConfigLog, (const cave_talk::LogLevel), (override));

    void HearConfigWheelSpeedControl(const cave_talk::PID &wheel_0_params, const cave_talk::PID &wheel_1_params, const cave_talk::PID &wheel_2_params, const cave_talk::PID &wheel_3_params)
    {
        TestPIDObject(pid_wsc_saved_0, wheel_0_params);
        TestPIDObject(pid_wsc_saved_1, wheel_1_params);
        TestPIDObject(pid_wsc_saved_2, wheel_2_params);
        TestPIDObject(pid_wsc_saved_3, wheel_3_params);
    }

    void HearConfigSteeringControl(const cave_talk::PID &turn_rate_params)
    {
        TestPIDObject(pid_sc_saved_trp, turn_rate_params);
    }
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

TEST(CaveTalkCppTests, SpeakListenOogaBooga)
{
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

TEST(CaveTalkCppTests, SpeakListenMovement)
{
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

TEST(CaveTalkCppTests, SpeakListenCameraMovement)
{
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

TEST(CaveTalkCppTests, SpeakListenLights)
{
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

TEST(CaveTalkCppTests, SpeakListenArm)
{
    std::shared_ptr<MockListenerCallbacks> mock_listen_callbacks = std::make_shared<MockListenerCallbacks>();
    cave_talk::Talker roverMouth(Send);
    cave_talk::Listener roverEars(Receive, mock_listen_callbacks);

    ring_buffer.Clear();

    ASSERT_EQ(CAVE_TALK_ERROR_NONE, roverMouth.SpeakArm(true));
    EXPECT_CALL(*mock_listen_callbacks.get(), HearArm(true)).Times(1);
    ASSERT_EQ(CAVE_TALK_ERROR_NONE, roverEars.Listen());

    ring_buffer.Clear();

    ASSERT_EQ(CAVE_TALK_ERROR_NONE, roverMouth.SpeakArm(false));
    EXPECT_CALL(*mock_listen_callbacks.get(), HearArm(false)).Times(1);
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

    cave_talk::Quaternion quat;
    quat.set_w(.2341402);
    quat.set_x(.00000234);
    quat.set_y(.6894444442);
    quat.set_z(0.00527000212);

    cave_talk::Imu IMU;
    IMU.mutable_accel()->CopyFrom(accel);
    IMU.mutable_gyro()->CopyFrom(gyro);
    IMU.mutable_quat()->CopyFrom(quat);

    cave_talk::Encoder encoder_0;
    encoder_0.set_rate_radians_per_second(3.141592652);
    encoder_0.set_total_pulses(100000);

    ASSERT_EQ(CAVE_TALK_ERROR_NONE, roverMouth.SpeakOdometry(IMU, encoder_0, encoder_0, encoder_0, encoder_0));

    imu_odometry_saved = IMU;
    encoder_odometry_saved_0 = encoder_0;
    encoder_odometry_saved_1 = encoder_0;
    encoder_odometry_saved_2 = encoder_0;
    encoder_odometry_saved_3 = encoder_0;
    ASSERT_EQ(CAVE_TALK_ERROR_NONE, roverEars.Listen());
}

TEST(CaveTalkCppTests, SpeakListenLog)
{
    std::shared_ptr<MockListenerCallbacks> mock_listen_callbacks = std::make_shared<MockListenerCallbacks>();
    cave_talk::Talker roverMouth(Send);
    cave_talk::Listener roverEars(Receive, mock_listen_callbacks);

    ring_buffer.Clear();

    const char hw[] = "Hello World!";

    ASSERT_EQ(CAVE_TALK_ERROR_NONE, roverMouth.SpeakLog(hw));
    EXPECT_CALL(*mock_listen_callbacks.get(), HearLog(testing::Eq(std::string(hw)))).Times(1);
    ASSERT_EQ(CAVE_TALK_ERROR_NONE, roverEars.Listen());

    ring_buffer.Clear();

    const char ooga_booga_msg[] = "Ooga Booga Ooga Booga Ooga Booga Ooga Booga Ooga Booga Ooga Booga Ooga Booga Ooga Booga!";
    ASSERT_EQ(CAVE_TALK_ERROR_NONE, roverMouth.SpeakLog(ooga_booga_msg));
    EXPECT_CALL(*mock_listen_callbacks.get(), HearLog(testing::Eq(std::string(ooga_booga_msg)))).Times(1);
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
    servo_test_zero.set_min_duty_cycle_percentage(540);
    servo_test_zero.set_max_duty_cycle_percentage(2560);
    servo_test_zero.set_center_duty_cycle_percentage(1576);

    ASSERT_EQ(CAVE_TALK_ERROR_NONE, roverMouth.SpeakConfigServoWheels(servo_test_zero, servo_test_zero, servo_test_zero, servo_test_zero));
    servo_configservowheels_saved_0 = servo_test_zero;
    servo_configservowheels_saved_1 = servo_test_zero;
    servo_configservowheels_saved_2 = servo_test_zero;
    servo_configservowheels_saved_3 = servo_test_zero;
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
    servo_test_zero.set_min_duty_cycle_percentage(540);
    servo_test_zero.set_max_duty_cycle_percentage(2560);
    servo_test_zero.set_center_duty_cycle_percentage(1576);

    ASSERT_EQ(CAVE_TALK_ERROR_NONE, roverMouth.SpeakConfigServoCams(servo_test_zero, servo_test_zero));
    servo_configservocams_saved_pan = servo_test_zero;
    servo_configservocams_saved_tilt = servo_test_zero;
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
    motor_configmotor_saved_0 = motor_test_zero;
    motor_configmotor_saved_1 = motor_test_zero;
    motor_configmotor_saved_2 = motor_test_zero;
    motor_configmotor_saved_3 = motor_test_zero;
    ASSERT_EQ(CAVE_TALK_ERROR_NONE, roverEars.Listen());
}

TEST(CaveTalkCppTests, SpeakListenConfigEncoder)
{
    std::shared_ptr<MockListenerCallbacks> mock_listen_callbacks = std::make_shared<MockListenerCallbacks>();
    cave_talk::Talker roverMouth(Send);
    cave_talk::Listener roverEars(Receive, mock_listen_callbacks);

    ring_buffer.Clear();

    cave_talk::ConfigEncoder config_encoder_test;
    config_encoder_test.set_smoothing_factor(.005);
    config_encoder_test.set_mode(cave_talk::EncoderMode::BSP_ENCODER_USER_MODE_PULSES_PER_ROTATON);
    config_encoder_test.set_pulses_per_period(6.2304);
    config_encoder_test.set_radians_per_pulse(3.000000001);

    ASSERT_EQ(CAVE_TALK_ERROR_NONE, roverMouth.SpeakConfigEncoder(config_encoder_test, config_encoder_test, config_encoder_test, config_encoder_test));
    configencoder_configencoder_saved_0 = config_encoder_test;
    configencoder_configencoder_saved_1 = config_encoder_test;
    configencoder_configencoder_saved_2 = config_encoder_test;
    configencoder_configencoder_saved_3 = config_encoder_test;
    ASSERT_EQ(CAVE_TALK_ERROR_NONE, roverEars.Listen());
}

TEST(CaveTalkCppTests, SpeakListenConfigLog)
{
    std::shared_ptr<MockListenerCallbacks> mock_listen_callbacks = std::make_shared<MockListenerCallbacks>();
    cave_talk::Talker roverMouth(Send);
    cave_talk::Listener roverEars(Receive, mock_listen_callbacks);

    ring_buffer.Clear();

    ASSERT_EQ(CAVE_TALK_ERROR_NONE, roverMouth.SpeakConfigLog(cave_talk::LogLevel::BSP_LOGGER_LEVEL_ERROR));
    EXPECT_CALL(*mock_listen_callbacks.get(), HearConfigLog(cave_talk::LogLevel::BSP_LOGGER_LEVEL_ERROR)).Times(1);
    ASSERT_EQ(CAVE_TALK_ERROR_NONE, roverEars.Listen());

    ring_buffer.Clear();

    ASSERT_EQ(CAVE_TALK_ERROR_NONE, roverMouth.SpeakConfigLog(cave_talk::LogLevel::BSP_LOGGER_LEVEL_WARNING));
    EXPECT_CALL(*mock_listen_callbacks.get(), HearConfigLog(cave_talk::LogLevel::BSP_LOGGER_LEVEL_WARNING)).Times(1);
    ASSERT_EQ(CAVE_TALK_ERROR_NONE, roverEars.Listen());

    ring_buffer.Clear();

    ASSERT_EQ(CAVE_TALK_ERROR_NONE, roverMouth.SpeakConfigLog(cave_talk::LogLevel::BSP_LOGGER_LEVEL_INFO));
    EXPECT_CALL(*mock_listen_callbacks.get(), HearConfigLog(cave_talk::LogLevel::BSP_LOGGER_LEVEL_INFO)).Times(1);
    ASSERT_EQ(CAVE_TALK_ERROR_NONE, roverEars.Listen());

    ring_buffer.Clear();

    ASSERT_EQ(CAVE_TALK_ERROR_NONE, roverMouth.SpeakConfigLog(cave_talk::LogLevel::BSP_LOGGER_LEVEL_DEBUG));
    EXPECT_CALL(*mock_listen_callbacks.get(), HearConfigLog(cave_talk::LogLevel::BSP_LOGGER_LEVEL_DEBUG)).Times(1);
    ASSERT_EQ(CAVE_TALK_ERROR_NONE, roverEars.Listen());

    ring_buffer.Clear();

    ASSERT_EQ(CAVE_TALK_ERROR_NONE, roverMouth.SpeakConfigLog(cave_talk::LogLevel::BSP_LOGGER_LEVEL_VERBOSE));
    EXPECT_CALL(*mock_listen_callbacks.get(), HearConfigLog(cave_talk::LogLevel::BSP_LOGGER_LEVEL_VERBOSE)).Times(1);
    ASSERT_EQ(CAVE_TALK_ERROR_NONE, roverEars.Listen());

    ring_buffer.Clear();

    ASSERT_EQ(CAVE_TALK_ERROR_NONE, roverMouth.SpeakConfigLog(cave_talk::LogLevel::BSP_LOGGER_LEVEL_MAX));
    EXPECT_CALL(*mock_listen_callbacks.get(), HearConfigLog(cave_talk::LogLevel::BSP_LOGGER_LEVEL_MAX)).Times(1);
    ASSERT_EQ(CAVE_TALK_ERROR_NONE, roverEars.Listen());
}

TEST(CaveTalkCppTests, SpeakListenConfigWheelSpeedControl)
{
    std::shared_ptr<MockListenerCallbacks> mock_listen_callbacks = std::make_shared<MockListenerCallbacks>();
    cave_talk::Talker roverMouth(Send);
    cave_talk::Listener roverEars(Receive, mock_listen_callbacks);

    ring_buffer.Clear();

    cave_talk::PID wheel_params;
    wheel_params.set_kp(1.03789);
    wheel_params.set_ki(.00000453);
    wheel_params.set_kd(1034798);

    ASSERT_EQ(CAVE_TALK_ERROR_NONE, roverMouth.SpeakConfigWheelSpeedControl(wheel_params, wheel_params, wheel_params, wheel_params));
    pid_wsc_saved_0 = wheel_params;
    pid_wsc_saved_1 = wheel_params;
    pid_wsc_saved_2 = wheel_params;
    pid_wsc_saved_3 = wheel_params;
    ASSERT_EQ(CAVE_TALK_ERROR_NONE, roverEars.Listen());
}

TEST(CaveTalkCppTests, SpeakListenConfigSteeringControl)
{
    std::shared_ptr<MockListenerCallbacks> mock_listen_callbacks = std::make_shared<MockListenerCallbacks>();
    cave_talk::Talker roverMouth(Send);
    cave_talk::Listener roverEars(Receive, mock_listen_callbacks);

    ring_buffer.Clear();

    cave_talk::PID wheel_params;
    wheel_params.set_kp(1.03789);
    wheel_params.set_ki(.00000453);
    wheel_params.set_kd(1034798);

    ASSERT_EQ(CAVE_TALK_ERROR_NONE, roverMouth.SpeakConfigSteeringControl(wheel_params));
    pid_sc_saved_trp = wheel_params;
    ASSERT_EQ(CAVE_TALK_ERROR_NONE, roverEars.Listen());
}