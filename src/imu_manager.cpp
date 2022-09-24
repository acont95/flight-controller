#include "imu_manager.h"

ImuManager::ImuManager(ICM20948_IMU& imu, mbed::InterruptIn& imu_int, events::EventQueue& event_queue, USBSerial& serial) : imu(imu), imu_int(imu_int), event_queue(event_queue), serial(serial){
    imu.setPwrMgmt1(DEVICE_RESET::DEVICE_RESET_REGISTERS, SLEEP::SLEEP_MODE_DISABLED, LP_EN::LOW_POWER_MODE_DISABLED, TEMP_DIS::TEMP_SENSOR_DISABLED, CLKSEL::AUTO_CLK);
    rtos::ThisThread::sleep_for(rtos::Kernel::Clock::duration_u32 {100});   
    imu.setUserCtrl(DMP_EN::DMP_DISABLED, FIFO_EN::FIFO_DISABLED, I2C_MST_EN::I2C_DISABLED, I2C_IF_DIS::SPI_MODE, DMP_RST::DMP_RST_DO_NOTHING, SRAM_RST::SRAM_RST_DO_NOTHING, I2C_MST_RST::I2C_RST_DO_NOTHING);
    imu.setPwrMgmt1(DEVICE_RESET::DEVICE_RESET_DO_NOTHING, SLEEP::SLEEP_MODE_DISABLED, LP_EN::LOW_POWER_MODE_DISABLED, TEMP_DIS::TEMP_SENSOR_DISABLED, CLKSEL::AUTO_CLK);

    // imu.setPwrMgmt2(GYRO_ENABLE, ACCEL_ENABLE);
    // imu.setIntPinCfg(INT1_LOGIC_HIGH, INT1_PUSH_PULL, INT1_HELD_UNTIL_CLEARED, INT_STATUS_READ_CLEAR, FSYNC_ACTIVE_HIGH, FSYNC_INT_DISABLE, I2C_MASTER_BYPASS_MODE_DISABLE);
    // imu.setIntEnable(DISABLE_WAKE_FSYNC, DISABLE_WAKE_MOTION, DISABLE_PLL_RDY_INT, DISABLE_DMP_INT, DISABLE_I2C_MASTER_INT);
    // imu.setIntEnable1(ENABLE_RAW_DATA_INT);
    // imu.setIntEnable2(ENABLE_INT_FIFO_OVERFLOW);
    // imu.setFifoEn2(ENABLE_ACCEL_FIFO, ENABLE_GYRO_Z_FIFO, ENABLE_GYRO_Y_FIFO, ENABLE_GYRO_X_FIFO, DISABLE_TEMP_FIFO);
    // imu.setFifoMode(FIFO_STREAM);
    // imu.setGyroSmplrtDiv(3);
    // imu.setGyroConfig1(GYRO_DLPF_1, GYRO_RANGE_2000, GYRO_ENABLE_DLPF);
    imu.setOdrAlignEn(ODR_ALIGN_EN::ENABLE_ODR_START_TIME_ALIGNMENT);
    // imu.setAccelSmplrtDiv(3);
    // imu.setAccelConfig(ACCEL_DLPFCFG_1, ACCEL_FULL_SCALE_4G, ACCEL_ENABLE_DLPF);
    // imu.ak09916Control2(CONTINUOUS_MEASUREMENT_MODE4);

    imu_int.rise(mbed::callback(this, &ImuManager::isr));
}

float ImuManager::getInertialRollAccel(xyz16Int xyz_body_accel) {
    float mag = sqrt(pow(xyz_body_accel.y, 2.0f) + pow(xyz_body_accel.z, 2.0f));
    return atan2f(-xyz_body_accel.x, mag) * 180 / (float) pi;
}

float ImuManager::getInertialPitchAccel(xyz16Int xyz_body_accel) {
    return atan2f(xyz_body_accel.y, xyz_body_accel.z) * 180 / (float) pi;
}

void ImuManager::updateAttitude() {
    accelGyroData data = imu.readFifo();
    
    float accel_pitch = getInertialPitchAccel(data.accel);
    float accel_roll = getInertialRollAccel(data.accel);

    float gyro_roll_rate = getInertialRollRateGyro(data.gyro);
    float gyro_pitch_rate = getInertialPitchRateGyro(data.gyro);
    float gyro_yaw_rate = getInertialYawRateGyro(data.gyro);

    float gyro_roll = getInertialRollGyro(gyro_roll_rate);
    float gyro_pitch = getInertialPitchGyro(gyro_pitch_rate);
    float gyro_yaw = getInertialYawGyro(gyro_yaw_rate);

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


float ImuManager::getInertialRollRateGyro(xyz16Int xyz_body_gyro) {
    return xyz_body_gyro.x + xyz_body_gyro.y * sinf(attitude.roll) * tanf(attitude.pitch) 
        + xyz_body_gyro.z * cosf(attitude.roll) * tanf(attitude.pitch);    
}

float ImuManager::getInertialPitchRateGyro(xyz16Int xyz_body_gyro) {
    return xyz_body_gyro.y * cosf(attitude.roll) - xyz_body_gyro.z * sinf(attitude.roll);
}

float ImuManager::getInertialYawRateGyro(xyz16Int xyz_body_gyro) {
    return xyz_body_gyro.y * sinf(attitude.roll) / cosf(attitude.pitch) 
        + xyz_body_gyro.z * cosf(attitude.roll) / cosf(attitude.pitch);
}

float ImuManager::getInertialRollGyro(float roll_rate) {
    return trapezoid(roll_rate, gyroAttitude.roll, dt);
}

float ImuManager::getInertialPitchGyro(float pitch_rate) {
    return trapezoid(pitch_rate, gyroAttitude.pitch, dt);
}

float ImuManager::getInertialYawGyro(float yaw_rate) {
    return trapezoid(yaw_rate, gyroAttitude.yaw, dt);
}

float ImuManager::trapezoid(float current_val, float previous_val, uint64_t delta) {
    return ((current_val + previous_val) / 2) * delta;
}

float ImuManager::complementaryFilter(float gyro_angle, float accel_angle) {
    return (1-alpha) * accel_angle + alpha * gyro_angle;
}

float ImuManager::getMagnetometerYaw(xyz16Int magnetometer_data) {
    float Bfy = (magnetometer_data.z - hardIronOffset.z)*sinf(attitude.roll) - (magnetometer_data.y - hardIronOffset.y)*sinf(attitude.roll);
    float Bfx = (magnetometer_data.x - hardIronOffset.x)*cosf(attitude.pitch) + (magnetometer_data.y - hardIronOffset.y)
        *sinf(attitude.roll)*sinf(attitude.pitch) + (magnetometer_data.z - hardIronOffset.z)*sinf(attitude.pitch)*cosf(attitude.roll);

    return atan2f(-Bfy, Bfx);
}

void ImuManager::isr() {
    event_queue.call(this, &ImuManager::interruptHandler);
}

void ImuManager::interruptHandler() {
    serial.printf("IMU Intterupt Triggered\n");
    switch (getInterruptType()) {
        case DATA_READY:    
            serial.printf("DATA_READY\n");

            timer.stop();
            setDt(timer.elapsed_time().count());
            timer.reset();
            timer.start();
            event_queue.call(this, &ImuManager::updateAttitude);
            break;
        case FIFO_OVERFLOW:
            serial.printf("FIFO_OVERFLOW\n");

            event_queue.call(this, &ImuManager::resetFifo);
            break;
        case NO_VALID_INTERRUPT:
            serial.printf("NO_VALID_INTERRUPT\n");

            break;
    }
    event_queue.call(this, &ImuManager::interruptClear);
}

uint64_t ImuManager::getDt() {
    return dt;
}

void ImuManager::testPrint(USBSerial& serial) {
    xyz16Int res = imu.accelData();
    serial.printf("X: %i\n", (int) res.x);
    serial.printf("Y: %i\n", (int) res.y);
    serial.printf("Z: %i\n", (int) res.z); 
    serial.printf("Pitch: %f\n", getInertialPitchAccel(res));
    serial.printf("Roll: %f\n", getInertialRollAccel(res));

    // serial.printf("Roll: %f\n", (float) attitude.roll);
    // serial.printf("Pitch: %f\n", (float) attitude.pitch);
    // serial.printf("Yaw: %f\n", (float) attitude.yaw); 
}

const xyzSigned16Int ImuManager::hardIronOffset = {.x=0, .y=0, .z=0};
const xyzSigned16Int ImuManager::softIronOffset = {.x=0, .y=0, .z=0};

const float ImuManager::pi = (float) M_PI;
const float ImuManager::alpha = 0.95f;