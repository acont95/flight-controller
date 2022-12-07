#pragma once

#include <stdio.h>
#include <pico/stdlib.h>
#include <icm20948_imu.hpp>
#include <sleep_interface.hpp>
#include <timer_interface.hpp>
// #include <rcl/rcl.h>
// #include <rclc/rclc.h>
// #include <error_loop.h>
// #include <sensor_msgs/msg/imu.h>
#include <quaternion.hpp>


#define M_PI		3.14159265358979323846
#define MS2_G       9.80665

enum INTERRUPT_TYPE {
    DATA_READY,
    FIFO_OVERFLOW,
    NO_VALID_INTERRUPT
};

struct Attitude {
    float yaw;
    float pitch;
    float roll;
};

struct XYZFloat {
    float x;
    float y;
    float z;
};

struct AccelGyroFloat {
    XYZFloat accel;
    XYZFloat gyro;
};

struct SoftIronOffset {
    XYZFloat row1;
    XYZFloat row2;
    XYZFloat row3;
};

class ImuManager {
    // Gyro reference: https://liqul.github.io/blog/assets/rotation.pdf
    // Accel reference: https://www.nxp.com/files-static/sensors/doc/app_note/AN3461.pdf

    public:
        ImuManager(ICM20948_IMU& imu, SleepInterface& sleeper, TimerInterface& timer, XYZ16Int hardIronOffset, SoftIronOffset softIronOffset, uint16_t sampleRateDiv);
        void updateAttitude();
        void updateAttitudeQuat();
        Attitude getAttitude();
        void testPrint();
        void init();
        void interruptHandler();
        void publishAttitude();
        void enableInterrupts();

    private:
        uint64_t getDt();
        INTERRUPT_TYPE getInterruptType();
        void interruptClear();
        void resetFifo();
        float getInertialRollAccel(XYZ16Int xyz_body_accel);
        float getInertialPitchAccel(XYZ16Int xyz_body_accel);
        float getInertialRollRateGyro(XYZ16Int xyz_body_gyro);
        float getInertialPitchRateGyro(XYZ16Int xyz_body_gyro);
        float getInertialYawRateGyro(XYZ16Int xyz_body_gyro);
        float getInertialRollGyro(float roll_rate);
        float getInertialPitchGyro(float pitch_rate);
        float getInertialYawGyro(float yaw_rate);
        float trapezoid(float current_val, float previous_val, float delta);
        float complementaryFilter(float gyro_angle, float accel_angle);
        float getMagnetometerYaw(XYZ16Int magnetometer_data);
        XYZFloat normalizeRaw(XYZ16Int raw);
        XYZFloat normalizeFloat(XYZFloat f);
        static const float alpha;
        ICM20948_IMU& imu;
        void setSampleRateDt(uint16_t sampleRateHz);
        float dt;
        uint16_t sampleRateDiv;
        Attitude gyroAttitude = {.yaw=0, .pitch=0, .roll=0};
        Attitude gyroRollRate = {.yaw=0, .pitch=0, .roll=0};
        Attitude attitude = {.yaw=0, .pitch=0, .roll=0};
        AccelGyroFloat sensorFrameData;
        SleepInterface& sleeper;
        TimerInterface& timer;
        uint32_t count = 0;
        const XYZ16Int hardIronOffset;
        const SoftIronOffset softIronOffset;
        Quaternion accelQuat;
        Quaternion magQuat;
        Quaternion gyroQuat = {.q0=1, .q1=0, .q2=0, .q3=0};
        Quaternion inertialQuat = {.q0=1, .q1=0, .q2=0, .q3=0};
        void setAccelQuat(XYZFloat xyz_body_accel_norm);
        void setGyroQuat(XYZFloat xyz_body_gyro_norm);
        void setMagQuat(Quaternion accel_quat, XYZFloat xyz_body_mag);
        void setInertialQuat();
        XYZFloat inverseRotate(Quaternion orientation, XYZFloat vec);
        float dcm[3][3];
        void dcmQuat(Quaternion quat);
        XYZFloat quatToEuler321(Quaternion q);
        XYZFloat applySoftIronOffset(XYZFloat mag);

        // rcl_publisher_t attitudePublisher;
};