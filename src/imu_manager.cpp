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
    imu.ak09916Control2(CONTINUOUS_MEASUREMENT_MODE4);
}

fix16_t ImuManager::getInertialRollAccel(xyz16Int xyz_body_accel) {
    float mag = sqrt(pow(xyz_body_accel.y, 2.0f) + pow(xyz_body_accel.z, 2.0f));
    return atan2f(-xyz_body_accel.x, mag);
}

fix16_t ImuManager::getInertialPitchAccel(xyz16Int xyz_body_accel) {
    return atan2f(xyz_body_accel.y, xyz_body_accel.z) * 180 / (float) pi;
}

void ImuManager::updateAttitude() {
    accelGyroData data = imu.readFifo();
    
    fix16_t accel_pitch = getInertialPitchAccel(data.accel);
    fix16_t accel_roll = getInertialRollAccel(data.accel);

    fix16_t gyro_roll_rate = getInertialRollRateGyro(data.gyro);
    fix16_t gyro_pitch_rate = getInertialPitchRateGyro(data.gyro);
    fix16_t gyro_yaw_rate = getInertialYawRateGyro(data.gyro);

    fix16_t gyro_roll = getInertialRollGyro(gyro_roll_rate);
    fix16_t gyro_pitch = getInertialPitchGyro(gyro_pitch_rate);
    fix16_t gyro_yaw = getInertialYawGyro(gyro_yaw_rate);

    gyroAttitude.roll += gyro_roll;
    gyroAttitude.pitch += gyro_pitch;
    gyroAttitude.yaw += gyro_yaw;

    attitude.pitch = complementaryFilter(gyroAttitude.pitch, accel_pitch);
    attitude.roll = complementaryFilter(gyroAttitude.roll, accel_roll);
    if (imu.ak09916Status1().DRDY) {
        xyz16Int magnetometer_data = imu.ak09916MeasurementData();
        attitude.yaw = getMagnetometerYaw(magnetometer_data);
    }
}

Attitude ImuManager::getAttitude() {
    return attitude;
}

void ImuManager::setDt(uint64_t dt) {
    this->dt = dt;
}

INTERRUPT_TYPE ImuManager::getInterruptType() {
    INT_STATUS_1 rawDataIntStatus = imu.intStatus1();
    INT_STATUS_2 overflowInt = imu.intStatus2();

    if (overflowInt.FIFO_OVERFLOW_INT) {
        return FIFO_OVERFLOW;
    } else if (rawDataIntStatus.RAW_DATA_0_RDY_INT) {
        return DATA_READY;
    } else {
        return NO_VALID_INTERRUPT;
    }
}

void ImuManager::resetFifo() {
    imu.fifoRst();
}

void ImuManager::interruptClear() {
    imu.intStatus();
}


fix16_t ImuManager::getInertialRollRateGyro(xyz16Int xyz_body_gyro) {
    return xyz_body_gyro.x + xyz_body_gyro.y * fix16_sin(attitude.roll) * fix16_tan(attitude.pitch) 
        + xyz_body_gyro.z * fix16_cos(attitude.roll) * fix16_tan(attitude.pitch);    
}

fix16_t ImuManager::getInertialPitchRateGyro(xyz16Int xyz_body_gyro) {
    return xyz_body_gyro.y * fix16_cos(attitude.roll) - xyz_body_gyro.z * fix16_sin(attitude.roll);
}

fix16_t ImuManager::getInertialYawRateGyro(xyz16Int xyz_body_gyro) {
    return xyz_body_gyro.y * fix16_sin(attitude.roll) / fix16_cos(attitude.pitch) 
        + xyz_body_gyro.z * fix16_cos(attitude.roll) / fix16_cos(attitude.pitch);
}

fix16_t ImuManager::getInertialRollGyro(fix16_t roll_rate) {
    return trapezoid(roll_rate, gyroAttitude.roll, dt);
}

fix16_t ImuManager::getInertialPitchGyro(fix16_t pitch_rate) {
    return trapezoid(pitch_rate, gyroAttitude.pitch, dt);
}

fix16_t ImuManager::getInertialYawGyro(fix16_t yaw_rate) {
    return trapezoid(yaw_rate, gyroAttitude.yaw, dt);
}

fix16_t ImuManager::trapezoid(fix16_t current_val, fix16_t previous_val, uint64_t delta) {
    return ((current_val + previous_val) / 2) * delta;
}

fix16_t ImuManager::complementaryFilter(fix16_t gyro_angle, fix16_t accel_angle) {
    return (1-alpha) * accel_angle + alpha * gyro_angle;
}

int32_t ImuManager::getMagnetometerYaw(xyz16Int magnetometer_data) {
    fix16_t Bfy = (magnetometer_data.z - hardIronOffset.z)*fix16_sin(attitude.roll) - (magnetometer_data.y - hardIronOffset.y)*fix16_sin(attitude.roll);
    fix16_t Bfx = (magnetometer_data.x - hardIronOffset.x)*fix16_cos(attitude.pitch) + (magnetometer_data.y - hardIronOffset.y)
        *fix16_sin(attitude.roll)*fix16_sin(attitude.pitch) + (magnetometer_data.z - hardIronOffset.z)*fix16_sin(attitude.pitch)*fix16_cos(attitude.roll);

    return fix16_atan2(-Bfy, Bfx);
}

const xyzSigned16Int ImuManager::hardIronOffset = {.x=0, .y=0, .z=0};
const xyzSigned16Int ImuManager::softIronOffset = {.x=0, .y=0, .z=0};

const float ImuManager::pi = (float) M_PI;
const float ImuManager::alpha = 0.95f;