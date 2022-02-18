#include <icm20948_imu.h>
#include "flight_controller.h"
#define M_PI		3.14159265358979323846



class ImuManager {
    public:
        ImuManager(ICM20948_IMU& imu);
        Attitude getAttitude();
        void setDt(uint64_t dt);

    private:
        const float filter_weight = 0.05;
        const float fusion_weight = 0.05;
        float getInertialRollAccel(xyz16Int xyz_body_accel);
        float getInertialPitchAccel(xyz16Int xyz_body_accel);
        float getInertialRollRateGyro(xyz16Int xyz_body_gyro);
        float getInertialPitchRateGyro(xyz16Int xyz_body_gyro);
        float getInertialYawRateGyro(xyz16Int xyz_body_gyro);
        float getInertialRollGyro(float roll_rate);
        float getInertialPitchGyro(float pitch_rate);
        float getInertialYawGyro(float yaw_rate);
        float trapezoid(float current_val, float previous_val, uint64_t delta);

        static const float pi = (float) M_PI;
        ICM20948_IMU& imu;
        uint64_t dt = 0;
        Attitude gyroRate = {.yaw=0, .pitch=0, .roll=0};
        Attitude previousAttitude = {.yaw=0, .pitch=0, .roll=0};
};