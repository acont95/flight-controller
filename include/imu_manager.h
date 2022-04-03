#pragma once

#include <icm20948_imu.h>
#include <libfixmath/fixmath.h>

#define M_PI		3.14159265358979323846

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
    int32_t yaw;
    int32_t pitch;
    int32_t roll;
};

class ImuManager {
    // Gyro reference: https://liqul.github.io/blog/assets/rotation.pdf
    // Accel reference: https://www.nxp.com/files-static/sensors/doc/app_note/AN3461.pdf

    public:
        ImuManager(ICM20948_IMU& imu, mbed::InterruptIn& imu_int, events::EventQueue& event_queue);
        void updateAttitude();
        void setDt(uint64_t dt);
        Attitude getAttitude();
        INTERRUPT_TYPE getInterruptType();
        void interruptClear();
        void resetFifo();

    private:
        fix16_t getInertialRollAccel(xyz16Int xyz_body_accel);
        fix16_t getInertialPitchAccel(xyz16Int xyz_body_accel);
        fix16_t getInertialRollRateGyro(xyz16Int xyz_body_gyro);
        fix16_t getInertialPitchRateGyro(xyz16Int xyz_body_gyro);
        fix16_t getInertialYawRateGyro(xyz16Int xyz_body_gyro);
        fix16_t getInertialRollGyro(fix16_t roll_rate);
        fix16_t getInertialPitchGyro(fix16_t pitch_rate);
        fix16_t getInertialYawGyro(fix16_t yaw_rate);
        fix16_t trapezoid(fix16_t current_val, fix16_t previous_val, uint64_t delta);
        fix16_t complementaryFilter(fix16_t gyro_angle, fix16_t accel_angle);
        fix16_t getMagnetometerYaw(xyz16Int magnetometer_data);
        void isr();

        static const float pi;
        static const float alpha;
        ICM20948_IMU& imu;
        uint64_t dt = 0;
        Attitude gyroAttitude = {.yaw=0, .pitch=0, .roll=0};
        Attitude attitude = {.yaw=0, .pitch=0, .roll=0};
        static const xyzSigned16Int hardIronOffset;
        static const xyzSigned16Int softIronOffset;
        mbed::InterruptIn& imu_int;
        events::EventQueue& event_queue;
        mbed::Timer timer;
};