#pragma once

#include <icm20948_imu.h>
#include <USBSerial.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <flight_controller_msgs/msg/imu_attitude.h>

#define M_PI		3.14159265358979323846

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

enum INTERRUPT_TYPE {
    DATA_READY,
    FIFO_OVERFLOW,
    NO_VALID_INTERRUPT
};

struct xyzSigned16Int {
    int16_t x;
    int16_t y;
    int16_t z;
};

struct Attitude {
    float yaw;
    float pitch;
    float roll;
};

class ImuManager {
    // Gyro reference: https://liqul.github.io/blog/assets/rotation.pdf
    // Accel reference: https://www.nxp.com/files-static/sensors/doc/app_note/AN3461.pdf

    public:
        ImuManager(ICM20948_IMU& imu, mbed::InterruptIn& imu_int, events::EventQueue& event_queue, USBSerial& serial);
        void updateAttitude();
        void setDt(uint64_t dt);
        uint64_t getDt();
        Attitude getAttitude();
        INTERRUPT_TYPE getInterruptType();
        void interruptClear();
        void resetFifo();
        void testPrint(USBSerial& serial);

    private:
        float getInertialRollAccel(xyz16Int xyz_body_accel);
        float getInertialPitchAccel(xyz16Int xyz_body_accel);
        float getInertialRollRateGyro(xyz16Int xyz_body_gyro);
        float getInertialPitchRateGyro(xyz16Int xyz_body_gyro);
        float getInertialYawRateGyro(xyz16Int xyz_body_gyro);
        float getInertialRollGyro(float roll_rate);
        float getInertialPitchGyro(float pitch_rate);
        float getInertialYawGyro(float yaw_rate);
        float trapezoid(float current_val, float previous_val, uint64_t delta);
        float complementaryFilter(float gyro_angle, float accel_angle);
        float getMagnetometerYaw(xyz16Int magnetometer_data);
        void isr();
        void interruptHandler();

        static const float pi;
        static const float alpha;
        ICM20948_IMU& imu;
        uint64_t dt = 0;
        Attitude gyroAttitude = {.yaw=0, .pitch=0, .roll=0};
        Attitude gyroRollRate = {.yaw=0, .pitch=0, .roll=0};
        Attitude attitude = {.yaw=0, .pitch=0, .roll=0};
        static const xyzSigned16Int hardIronOffset;
        static const xyzSigned16Int softIronOffset;
        mbed::InterruptIn& imu_int;
        events::EventQueue& event_queue;
        mbed::Timer timer;
        USBSerial& serial;
        uint32_t count = 0;

        rclc_support_t support;
        rcl_allocator_t allocator;
        rcl_node_t node;
        rcl_publisher_t attitudePublisher;
};