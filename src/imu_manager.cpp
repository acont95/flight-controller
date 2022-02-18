#include "imu_manager.h"

ImuManager::ImuManager(ICM20948_IMU& imu) : imu(imu){
    imu.setUserCtrl(DMP_DISABLED, FIFO_ENABLED, I2C_DISABLED, SPI_MODE, DMP_RST_DO_NOTHING, SRAM_RST_DO_NOTHING, I2C_RST_DO_NOTHING);
    imu.setPwrMgmt1(DEVICE_RESET_DO_NOTHING, SLEEP_MODE_DISABLED, LOW_POWER_MODE_DISABLED, TEMP_SENSOR_DISABLED, AUTO_CLK);
    imu.setPwrMgmt2(GYRO_ENABLE, ACCEL_ENABLE);
    imu.setIntPinCfg(INT1_LOGIC_HIGH, INT1_PUSH_PULL, INT1_HELD_UNTIL_CLEARED, INT_STATUS_READ_CLEAR, FSYNC_ACTIVE_HIGH, FSYNC_INT_DISABLE, I2C_MASTER_BYPASS_MODE_DISABLE);
    imu.setIntEnable(DISABLE_WAKE_FSYNC, DISABLE_WAKE_MOTION, DISABLE_PLL_RDY_INT, DISABLE_DMP_INT, DISABLE_I2C_MASTER_INT);
    imu.setIntEnable1(ENABLE_RAW_DATA_INT);
    imu.setIntEnable2(ENABLE_INT_FIFO_OVERFLOW);
    imu.setFifoEn2(ENABLE_ACCEL_FIFO, ENABLE_GYRO_Z_FIFO, ENABLE_GYRO_Y_FIFO, ENABLE_GYRO_X_FIFO, DISABLE_TEMP_FIFO);
    imu.setFifoMode(FIFO_STREAM);
    imu.setGyroSmplrtDiv(3);
    imu.setGyroConfig1(GYRO_DLPF_0, GYRO_RANGE_1000, GYRO_ENABLE_DLPF);
    imu.setOdrAlignEn(ENABLE_ODR_START_TIME_ALIGNMENT);
    imu.setAccelSmplrtDiv(3);
    imu.setAccelConfig(ACCEL_DLPFCFG_0, ACCEL_FULL_SCALE_16G, ACCEL_ENABLE_DLPF);
}

float ImuManager::getInertialRollAccel(xyz16Int xyz_body_accel) {
    float mag = sqrt(pow(xyz_body_accel.y, 2.0f) + pow(xyz_body_accel.z, 2.0f));
    return atan2f(-xyz_body_accel.x, mag);
}

float ImuManager::getInertialPitchAccel(xyz16Int xyz_body_accel) {
    return atan2f(xyz_body_accel.y, xyz_body_accel.z) * 180 / (float) pi;
}

Attitude ImuManager::getAttitude() {
    accelGyroData data = imu.readFifo();
    Attitude attitude;
    
    float accel_pitch = getInertialPitchAccel(data.accel);
    float accel_roll = getInertialRollAccel(data.accel);

    float gyro_roll_rate = getInertialRollRateGyro(data.gyro);
    float gyro_pitch_rate = getInertialPitchRateGyro(data.gyro);
    float gyro_yaw_rate = getInertialYawRateGyro(data.gyro);

    float gyro_roll = getInertialRollGyro(gyro_roll_rate);
    float gyro_pitch = getInertialPitchGyro(gyro_pitch_rate);
    float gyro_yaw = getInertialYawGyro(gyro_yaw_rate);

    gyroRate.roll += gyro_roll;
    gyroRate.pitch += gyro_pitch;
    gyroRate.yaw += gyro_yaw;


    // attitude.pitch = 
    // attitude.roll = 
    // attitude.yaw = 

    return attitude;
}

void ImuManager::setDt(uint64_t dt) {
    this->dt = dt;
}


float ImuManager::getInertialRollRateGyro(xyz16Int xyz_body_gyro) {
    return xyz_body_gyro.x + xyz_body_gyro.y * sinf(previousAttitude.roll) * tanf(previousAttitude.pitch) 
        + xyz_body_gyro.z * cosf(previousAttitude.roll) * tanf(previousAttitude.pitch);    
}

float ImuManager::getInertialPitchRateGyro(xyz16Int xyz_body_gyro) {
    return xyz_body_gyro.y * cosf(previousAttitude.roll) - xyz_body_gyro.z * sinf(previousAttitude.roll);
}

float ImuManager::getInertialYawRateGyro(xyz16Int xyz_body_gyro) {
    return xyz_body_gyro.y * sinf(previousAttitude.roll) / cosf(previousAttitude.pitch) 
        + xyz_body_gyro.z * cosf(previousAttitude.roll) / cosf(previousAttitude.pitch);
}

float ImuManager::getInertialRollGyro(float roll_rate) {
    return trapezoid(roll_rate, gyroRate.roll, dt);
}

float ImuManager::getInertialPitchGyro(float pitch_rate) {
    return trapezoid(pitch_rate, gyroRate.pitch, dt);
}

float ImuManager::getInertialYawGyro(float yaw_rate) {
    return trapezoid(yaw_rate, gyroRate.yaw, dt);
}

float ImuManager::trapezoid(float current_val, float previous_val, uint64_t delta) {
    return ((current_val + previous_val) / 2) * delta;
}