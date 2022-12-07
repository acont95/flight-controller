#include "imu_manager.hpp"

ImuManager::ImuManager(
    ICM20948_IMU& imu, 
    SleepInterface& sleeper, 
    TimerInterface& timer, 
    XYZ16Int hardIronOffset, 
    SoftIronOffset softIronOffset,
    uint16_t sampleRateDiv
) : imu(imu), sleeper(sleeper), timer(timer), hardIronOffset(hardIronOffset), softIronOffset(softIronOffset), sampleRateDiv(sampleRateDiv){
    setSampleRateDt(sampleRateDiv);
}

void ImuManager::init() {
    imu.setPwrMgmt1(DEVICE_RESET::DEVICE_RESET_REGISTERS, SLEEP::SLEEP_MODE_DISABLED, LP_EN::LOW_POWER_MODE_DISABLED, TEMP_DIS::TEMP_SENSOR_DISABLED, CLKSEL::AUTO_CLK);
    sleeper.sleepMs(100);
    imu.setPwrMgmt1(DEVICE_RESET::DEVICE_RESET_DO_NOTHING, SLEEP::SLEEP_MODE_DISABLED, LP_EN::LOW_POWER_MODE_DISABLED, TEMP_DIS::TEMP_SENSOR_DISABLED, CLKSEL::AUTO_CLK);

    imu.setOdrAlignEn(ODR_ALIGN_EN::ENABLE_ODR_START_TIME_ALIGNMENT);
    imu.setAccelConfig(ACCEL_DLPFCFG::ACCEL_DLPFCFG_6, ACCEL_FS_SEL::ACCEL_FULL_SCALE_2G, ACCEL_FCHOICE::ACCEL_ENABLE_DLPF);
    imu.setAccelSmplrtDiv(sampleRateDiv);
    imu.setGyroConfig1(GYRO_DLPFCFG::GYRO_DLPF_6, GYRO_FS_SEL::GYRO_RANGE_1000, GYRO_FCHOICE::GYRO_ENABLE_DLPF);
    imu.setGyroSmplrtDiv(sampleRateDiv);
    imu.setGyroOffsets();

    imu.setIntPinCfg(
        INT1_ACTL::INT1_LOGIC_HIGH, 
        INT1_OPEN::INT1_PUSH_PULL, 
        INT1_LATCH__EN::INT1_HELD_UNTIL_CLEARED, 
        INT1_ANYRD_2CLEAR::INT_STATUS_READ_CLEAR, 
        ACTL_FSYNC::FSYNC_ACTIVE_HIGH, 
        FSYNC_INT_MODE_EN::FSYNC_INT_DISABLE, 
        BYPASS_EN::I2C_MASTER_BYPASS_MODE_DISABLE
    );

    // imu.setIntEnable(REG_WOF_EN::DISABLE_WAKE_FSYNC, WOM_INT_EN::DISABLE_WAKE_MOTION, PLL_RDY_EN::DISABLE_PLL_RDY_INT, DMP_INT1_EN::DISABLE_DMP_INT, I2C_MST_INT_EN::ENABLE_I2C_MASTER_INT);

    imu.setUserCtrl(DMP_EN::DMP_DISABLED, FIFO_EN::FIFO_ENABLED, I2C_MST_EN::I2C_DISABLED, I2C_IF_DIS::SPI_MODE, DMP_RST::DMP_RST_DO_NOTHING, SRAM_RST::SRAM_RST_DO_NOTHING, I2C_MST_RST::I2C_RST_DO_NOTHING);
    imu.ak09916Init();
    // printf("MAG WHO AM I: %i\n", imu.getAk09916WhoAmI());
    imu.setupAk09916MeasurementData();

    imu.setFifoEn1(SLV_3_FIFO_EN::DISABLE_WRITE_TO_SLV_3, SLV_2_FIFO_EN::DISABLE_WRITE_TO_SLV_2, SLV_1_FIFO_EN::DISABLE_WRITE_TO_SLV_1,
        SLV_0_FIFO_EN::ENABLE_WIRTE_TO_SLV_0);
    imu.setFifoEn2(ACCEL_FIFO_EN::ENABLE_ACCEL_FIFO, GYRO_Z_FIFO_EN::ENABLE_GYRO_Z_FIFO, GYRO_Y_FIFO_EN::ENABLE_GYRO_Y_FIFO, GYRO_X_FIFO_EN::ENABLE_GYRO_X_FIFO, TEMP_FIFO_EN::DISABLE_TEMP_FIFO);

    // create publisher
    // RCCHECK(rclc_publisher_init_default(
    //     &attitudePublisher,
    //     &node,
    //     ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
    //     "/mcu/imu"
    //     )
    // );
}

float ImuManager::getInertialRollAccel(XYZ16Int xyz_body_accel) {
    float mag = sqrt(pow(xyz_body_accel.x, 2.0f) + pow(xyz_body_accel.z, 2.0f));
    return -atan2f(-xyz_body_accel.y, mag) * 180 / (float) M_PI;
}

float ImuManager::getInertialPitchAccel(XYZ16Int xyz_body_accel) {
    return -atan2f(xyz_body_accel.x, xyz_body_accel.z) * 180 / (float) M_PI;
}

void ImuManager::updateAttitude() {
    AccelGyroData data = imu.readFifo();
    // AccelGyroMagData data = imu.readFifoMag();

    float accel_pitch = getInertialPitchAccel(data.accel);
    float accel_roll = getInertialRollAccel(data.accel);

    GYRO_FS_SEL gyro_fs_sel = imu.getGyroConfig1().gyro_fs_sel;
    float gyro_scale_factor = imu.getGyroScaleFactor(gyro_fs_sel);

    data.gyro.x /= gyro_scale_factor;
    data.gyro.y /= gyro_scale_factor;
    data.gyro.z /= gyro_scale_factor;

    float gyro_roll_rate = getInertialRollRateGyro(data.gyro);
    float gyro_pitch_rate = getInertialPitchRateGyro(data.gyro);
    float gyro_yaw_rate = getInertialYawRateGyro(data.gyro);

    float gyro_roll = getInertialRollGyro(gyro_roll_rate);
    float gyro_pitch = getInertialPitchGyro(gyro_pitch_rate);
    float gyro_yaw = getInertialYawGyro(gyro_yaw_rate);

    gyroRollRate.roll = gyro_roll_rate;
    gyroRollRate.pitch = gyro_pitch_rate;
    gyroRollRate.yaw = gyro_yaw_rate;

    gyroAttitude.roll += gyro_roll;
    gyroAttitude.pitch += gyro_pitch;
    gyroAttitude.yaw += gyro_yaw;

    attitude.pitch = complementaryFilter(gyroAttitude.pitch, accel_pitch);
    attitude.roll = complementaryFilter(gyroAttitude.roll, accel_roll);

    count++;
    if ((count % 100) == 0) {
        // setAccelQuat(normalizeRaw(data.accel));
        // publishAttitude();
        printf("FIFO Count: %i\n", imu.getFifoCount());
        printf("Gyro X: %i\n", (int)data.gyro.x);
        printf("Gyro Y: %i\n", (int)data.gyro.y);
        printf("Gyro Z: %i\n", (int)data.gyro.z);

        printf("Accel X: %i\n", (int)data.accel.x);
        printf("Accel Y: %i\n", (int)data.accel.y);
        printf("Accel Z: %i\n", (int)data.accel.z);
    }

    // if (imu.ak09916Status1().DRDY) {
    //     xyz16Int magnetometer_data = imu.ak09916MeasurementData();
    //     attitude.yaw = getMagnetometerYaw(magnetometer_data);
    // }
}

void ImuManager::updateAttitudeQuat() {
    AccelGyroMagData data = imu.readFifoAllSensors();

    // Transform axes to NED coordinate system. 

    data.mag_data.mag.x -= hardIronOffset.x;
    data.mag_data.mag.y -= hardIronOffset.y;
    data.mag_data.mag.z -= hardIronOffset.z;

    XYZFloat calibratedMag = applySoftIronOffset({
        .x=(float) data.mag_data.mag.x,
        .y=(float) data.mag_data.mag.y,
        .z=(float) data.mag_data.mag.z
    });

    calibratedMag.y *= -1;
    calibratedMag.z *= -1;

    // printf("%i, %i, %i\n", data.mag_data.mag.x, data.mag_data.mag.y, data.mag_data.mag.z);

    // printf("%i, %i, %i\n", data.accel.x, data.accel.y, data.accel.z);

    GYRO_FS_SEL gyro_fs_sel = imu.getGyroConfig1().gyro_fs_sel;
    float gyro_scale_factor = imu.getGyroScaleFactor(gyro_fs_sel);

    ACCEL_FS_SEL accel_fs_sel = imu.getAccelConfig().accel_fs_sel;
    float accel_scale_factor = imu.getAccelScaleFactor(accel_fs_sel);

    sensorFrameData.accel.x = (float) data.accel.x * MS2_G / accel_scale_factor;
    sensorFrameData.accel.y = (float) data.accel.y * MS2_G / accel_scale_factor;
    sensorFrameData.accel.z = (float) data.accel.z * MS2_G / accel_scale_factor;

    sensorFrameData.gyro.x = (float) data.gyro.x / gyro_scale_factor;
    sensorFrameData.gyro.y = (float) data.gyro.y / gyro_scale_factor;
    sensorFrameData.gyro.z = (float) data.gyro.z / gyro_scale_factor;

    setAccelQuat(normalizeRaw(data.accel));
    setMagQuat(accelQuat, normalizeFloat(calibratedMag));

    XYZFloat gyro_float_rad = {
        .x=sensorFrameData.gyro.x * M_PI/180,
        .y=sensorFrameData.gyro.y * M_PI/180,
        .z=sensorFrameData.gyro.z * M_PI/180
    };
    setGyroQuat(gyro_float_rad);
    setInertialQuat();

    if ((count % 100) == 0) {
        // printf("Ang V Quat q0: %f\n", angular_vel_quat.q0);
        // printf("Ang V Quat q1: %f\n", angular_vel_quat.q1);
        // printf("Ang V Quat q2: %f\n", angular_vel_quat.q2);
        // printf("Ang V Quat q3: %f\n", angular_vel_quat.q3);

        // printf("Q1 Quat q0: %f\n", q1.q0);
        // printf("Q1 Quat q1: %f\n", q1.q1);
        // printf("Q1 Quat q2: %f\n", q1.q2);
        // printf("Q1 Quat q3: %f\n", q1.q3);

        // printf("Gyro Rate Quat q0: %f\n", gyroRateQuat.q0);
        // printf("Gyro Rate Quat q1: %f\n", gyroRateQuat.q1);
        // printf("Gyro Rate Quat q2: %f\n", gyroRateQuat.q2);
        // printf("Gyro Rate Quat q3: %f\n", gyroRateQuat.q3);
    
        // printf("Gyro Quat q0: %f\n", gyroQuat.q0);
        // printf("Gyro Quat q1: %f\n", gyroQuat.q1);
        // printf("Gyro Quat q2: %f\n", gyroQuat.q2);
        // printf("Gyro Quat q3: %f\n", gyroQuat.q3);

        // XYZFloat gyroEuler = quatToEuler321(gyroQuat);
        // printf("Gyro Euler x: %f\n", gyroEuler.x * 180/M_PI);
        // printf("Gyro Euler y: %f\n", gyroEuler.y * 180/M_PI);
        // printf("Gyro Euler z: %f\n", gyroEuler.z * 180/M_PI);
    
        // printf("Inertial Quat q0: %f\n", inertialQuat.q0);
        // printf("Inertial Quat q1: %f\n", inertialQuat.q1);
        // printf("Inertial Quat q2: %f\n", inertialQuat.q2);
        // printf("Inertial Quat q3: %f\n", inertialQuat.q3);

        // printf("Accel Quat q0: %f\n", accelQuat.q0);
        // printf("Accel Quat q1: %f\n", accelQuat.q1);
        // printf("Accel Quat q2: %f\n", accelQuat.q2);
        // printf("Accel Quat q3: %f\n", accelQuat.q3);
        // XYZFloat accelEuler = quatToEuler321(accelQuat);
        // printf("Accel Euler x: %f\n", accelEuler.x * 180/M_PI);
        // printf("Accel Euler y: %f\n", accelEuler.y * 180/M_PI);
        // printf("Accel Euler z: %f\n", accelEuler.z * 180/M_PI);

        // printf("Mag Quat q0: %f\n", magQuat.q0);
        // printf("Mag Quat q1: %f\n", magQuat.q1);
        // printf("Mag Quat q2: %f\n", magQuat.q2);
        // printf("Mag Quat q3: %f\n", magQuat.q3);
        // XYZFloat magEuler = quatToEuler321(magQuat);
        // printf("Mag Euler x: %f\n", magEuler.x * 180/M_PI);
        // printf("Mag Euler y: %f\n", magEuler.y * 180/M_PI);
        // printf("Mag Euler z: %f\n", magEuler.z * 180/M_PI);

        // Quaternion q1 = quat_mult(accelQuat, magQuat);
        // q1 = quat_conjugate(q1);

        // printf("Accel Mag Quat q0: %f\n", q1.q0);
        // printf("Accel Mag Quat q1: %f\n", q1.q1);
        // printf("Accel Mag Quat q2: %f\n", q1.q2);   
        // printf("Accel Mag Quat q3: %f\n", q1.q3);
        // XYZFloat accelMagEuler = quatToEuler321(q1);
        // printf("Accel Mag Euler x: %f\n", accelMagEuler.x * 180/M_PI);
        // printf("Accel Mag Euler y: %f\n", accelMagEuler.y * 180/M_PI);
        // printf("Accel Mag Euler z: %f\n", accelMagEuler.z * 180/M_PI);

        // printf("Inertial Quat q0: %f\n", inertialQuat.q0);
        // printf("Inertial Quat q1: %f\n", inertialQuat.q1);
        // printf("Inertial Quat q2: %f\n", inertialQuat.q2);
        // printf("Inertial Quat q3: %f\n", inertialQuat.q3);

        Quaternion q1 = quat_conjugate(inertialQuat);
        XYZFloat inertialEuler = quatToEuler321(q1);
        printf("Inertial Euler x: %f\n", inertialEuler.x * 180/M_PI);
        printf("Inertial Euler y: %f\n", inertialEuler.y * 180/M_PI);
        printf("Inertial Euler z: %f\n", inertialEuler.z * 180/M_PI);
    }
    count++;
}

Attitude ImuManager::getAttitude() {
    return attitude;
}

INTERRUPT_TYPE ImuManager::getInterruptType() {
    IntStatus1 rawDataIntStatus = imu.getIntStatus1();
    IntStatus2 overflowInt = imu.getIntStatus2();

    if (overflowInt.FIFO_OVERFLOW_INT) {
        return FIFO_OVERFLOW;
    } else if (rawDataIntStatus.RAW_DATA_0_RDY_INT) {
        return DATA_READY;
    } else {
        return NO_VALID_INTERRUPT;
    }
}

void ImuManager::resetFifo() {
    imu.setFifoRst();
}

void ImuManager::interruptClear() {
    imu.getIntStatus();
    imu.getIntStatus1();
    imu.getIntStatus2();
    imu.getIntStatus3();
}

float ImuManager::getInertialRollRateGyro(XYZ16Int xyz_body_gyro) {
    return xyz_body_gyro.x + xyz_body_gyro.y * sinf(gyroAttitude.roll * M_PI / 180) * tanf(gyroAttitude.pitch  * M_PI / 180) 
        + xyz_body_gyro.z * cosf(gyroAttitude.roll  * M_PI / 180) * tanf(gyroAttitude.pitch  * M_PI / 180);    
}

float ImuManager::getInertialPitchRateGyro(XYZ16Int xyz_body_gyro) {
    return xyz_body_gyro.y * cosf(gyroAttitude.roll  * M_PI / 180) - xyz_body_gyro.x * sinf(gyroAttitude.roll  * M_PI / 180);
}

float ImuManager::getInertialYawRateGyro(XYZ16Int xyz_body_gyro) {
    return xyz_body_gyro.y * sinf(attitude.roll) / cosf(attitude.pitch) 
        + xyz_body_gyro.z * cosf(attitude.roll) / cosf(attitude.pitch);
}

float ImuManager::getInertialRollGyro(float roll_rate) {
    return trapezoid(roll_rate, gyroRollRate.roll, dt);
}

float ImuManager::getInertialPitchGyro(float pitch_rate) {
    return trapezoid(pitch_rate, gyroRollRate.pitch, dt);
}

float ImuManager::getInertialYawGyro(float yaw_rate) {
    return trapezoid(yaw_rate, gyroRollRate.yaw, dt);
}

float ImuManager::trapezoid(float current_val, float previous_val, float delta) {
    return ((current_val + previous_val) / 2) * delta;
}

float ImuManager::complementaryFilter(float gyro_angle, float accel_angle) {
    return (1-alpha) * gyro_angle + alpha * accel_angle;
}

float ImuManager::getMagnetometerYaw(XYZ16Int magnetometer_data) {
    float Bfy = (magnetometer_data.z + hardIronOffset.z)*sinf(attitude.roll) - (magnetometer_data.y + hardIronOffset.y)*sinf(attitude.roll);
    float Bfx = (magnetometer_data.x + hardIronOffset.x)*cosf(attitude.pitch) + (magnetometer_data.y + hardIronOffset.y)
        *sinf(attitude.roll)*sinf(attitude.pitch) + (magnetometer_data.z + hardIronOffset.z)*sinf(attitude.pitch)*cosf(attitude.roll);

    return atan2f(-Bfy, Bfx);
}

void ImuManager::interruptHandler() {
    switch (getInterruptType()) {
        case DATA_READY:    
            // timer.stop();
            // setDt(timer.elapsedUs());
            // timer.start();
            while (!imu.getDataRdyStatus().RAW_DATA_RDY) {};
            updateAttitudeQuat();
            break;
        case FIFO_OVERFLOW:
            resetFifo();
            break;
        case NO_VALID_INTERRUPT:
            break;
    }
    interruptClear();
}

uint64_t ImuManager::getDt() {
    return dt;
}

void ImuManager::testPrint() {
    AK09916Data mag = imu.readAk09916ExtData();
    while (!mag.dataReady || mag.overflow) {
        mag = imu.readAk09916ExtData();
        sleep_ms(1);
    }
    // mag.mag.x += hardIronOffset.x;
    // mag.mag.y += hardIronOffset.y;
    // mag.mag.z += hardIronOffset.z;

    printf("%i, %i, %i\n", mag.mag.x, mag.mag.y, mag.mag.z);
    // printf("Angle: %f\n", atan2f((float)mag.mag.y, (float)mag.mag.x) * 180 / M_PI); 
}

void ImuManager::publishAttitude() {
    // geometry_msgs__msg__Quaternion orientation = {
    //     .x = accelQuat.q1,
    //     .y = accelQuat.q2,
    //     .z = accelQuat.q3,
    //     .w = accelQuat.q0
    // };

    // geometry_msgs__msg__Vector3 angular_velocity = {
    //     .x = (float) sensorFrameData.gyro.x,
    //     .y = (float) sensorFrameData.gyro.y,
    //     .z = (float) sensorFrameData.gyro.z
    // };

    // geometry_msgs__msg__Vector3 linear_accel = {
    //     .x = (float) sensorFrameData.accel.x,
    //     .y = (float) sensorFrameData.accel.y,
    //     .z = (float) sensorFrameData.accel.z
    // };

    // sensor_msgs__msg__Imu msg = {
    //     .orientation = orientation,
    //     .angular_velocity = angular_velocity,
    //     .linear_acceleration = linear_accel
    // };

    // RCSOFTCHECK(rcl_publish(&attitudePublisher, &msg, NULL));
}

XYZFloat ImuManager::normalizeRaw(XYZ16Int raw) {
    float x = (float) raw.x;
    float y = (float) raw.y;
    float z = (float) raw.z;
    float norm = sqrtf(x*x + y*y + z*z);
    XYZFloat normalized = {
        .x = (float) raw.x / norm,
        .y = (float) raw.y / norm,
        .z = (float) raw.z / norm
    };
    
    return normalized;
}

XYZFloat ImuManager::normalizeFloat(XYZFloat raw) {
    float norm = sqrtf(raw.x*raw.x + raw.y*raw.y + raw.z*raw.z);
    XYZFloat normalized = {
        .x = (float) raw.x / norm,
        .y = (float) raw.y / norm,
        .z = (float) raw.z / norm
    };
    
    return normalized;
}

void ImuManager::setAccelQuat(XYZFloat xyz_body_accel_norm) {
    switch (xyz_body_accel_norm.z >= 0)
    {
    case 1:
        accelQuat.q0 = sqrtf((xyz_body_accel_norm.z + 1)/2);
        accelQuat.q1 = - xyz_body_accel_norm.y / sqrtf(2*(xyz_body_accel_norm.z + 1));
        accelQuat.q2 = xyz_body_accel_norm.x / sqrtf(2*(xyz_body_accel_norm.z + 1));
        accelQuat.q3 = 0;
        break;
    
    case 0:
        accelQuat.q0 = - xyz_body_accel_norm.y / sqrtf(2*(1 - xyz_body_accel_norm.z));
        accelQuat.q1 = sqrtf((1 - xyz_body_accel_norm.z)/2);
        accelQuat.q2 = 0;
        accelQuat.q3 = xyz_body_accel_norm.x / sqrtf(2*(1 - xyz_body_accel_norm.z));
        break;
    }
    // accelQuat = quat_normalize(accelQuat);
}

void ImuManager::setGyroQuat(XYZFloat xyz_body_gyro_norm) {
    Quaternion angular_vel_quat = {
        .q0 = 0,
        .q1 = xyz_body_gyro_norm.x,
        .q2 = xyz_body_gyro_norm.y,
        .q3 = xyz_body_gyro_norm.z
    };
    Quaternion gyroRateQuat = quat_mult_scalar(quat_mult(angular_vel_quat, inertialQuat), -0.5f);
    gyroQuat = quat_sum(inertialQuat, quat_mult_scalar(gyroRateQuat, dt));
    gyroQuat = quat_normalize(gyroQuat);
}

void ImuManager::setMagQuat(Quaternion accel_quat, XYZFloat xyz_body_mag) {
    XYZFloat l = inverseRotate(accel_quat, xyz_body_mag); 
    float gamma = l.x*l.x + l.y*l.y;
    switch (l.x >= 0)
    {
    case 1:
        magQuat.q0 = sqrtf(gamma + l.x*sqrtf(gamma)) / sqrtf(2*gamma);
        magQuat.q1 = 0;
        magQuat.q2 = 0;
        magQuat.q3 = l.y / (sqrtf(2)*sqrtf(gamma + l.x*sqrtf(gamma)));
        break;
    
    case 0:
        magQuat.q0 = l.y / (sqrtf(2)*sqrtf(gamma - l.x*sqrtf(gamma)));
        magQuat.q1 = 0;
        magQuat.q2 = 0;
        magQuat.q3 = sqrtf(gamma - l.x*sqrtf(gamma)) / sqrtf(2*gamma);
        break;
    }
    // magQuat = quat_normalize(magQuat);
}

void ImuManager::setInertialQuat() {
    inertialQuat = quat_sum(quat_mult_scalar(gyroQuat, 1-0.95f), quat_mult_scalar(quat_mult(accelQuat, magQuat), 0.95));
    inertialQuat = quat_normalize(inertialQuat);
}

XYZFloat ImuManager::inverseRotate(Quaternion orientation, XYZFloat vec) {
    XYZFloat result;
    dcmQuat(orientation);   
    result.x = dcm[0][0]*vec.x + dcm[1][0]*vec.y + dcm[2][0]*vec.z;
    result.y = dcm[0][1]*vec.x + dcm[1][1]*vec.y + dcm[2][1]*vec.z;
    result.z = dcm[0][2]*vec.x + dcm[1][2]*vec.y + dcm[2][2]*vec.z;

    return result;
}

void ImuManager::dcmQuat(Quaternion q) {
    dcm[0][0] = q.q0*q.q0 + q.q1*q.q1 - q.q2*q.q2 - q.q3*q.q3;
    dcm[0][1] = 2*(q.q1*q.q2 - q.q0*q.q3);
    dcm[0][2] = 2*(q.q1*q.q3 + q.q0*q.q2);
    dcm[1][0] = 2*(q.q1*q.q2 + q.q0*q.q3);
    dcm[1][1] = q.q0*q.q0 - q.q1*q.q1 + q.q2*q.q2 - q.q3*q.q3;
    dcm[1][2] = 2*(q.q2*q.q3 - q.q0*q.q1);
    dcm[2][0] = 2*(q.q1*q.q3 - q.q0*q.q2);
    dcm[2][1] = 2*(q.q2*q.q3 + q.q0*q.q1);
    dcm[2][2] = q.q0*q.q0 - q.q1*q.q1 - q.q2*q.q2 + q.q3*q.q3;
}

void ImuManager::setSampleRateDt(uint16_t sample_rate_div) {
    float sampleRateHz = 1125.0f/(1+sample_rate_div);
    dt = 1.0f/sampleRateHz;
}

void ImuManager::enableInterrupts() {
    imu.setIntEnable1(RAW_DATA_0_RDY_EN::ENABLE_RAW_DATA_INT);
    imu.setIntEnable2(FIFO_OVERFLOW_EN::ENABLE_INT_FIFO_OVERFLOW);
}

XYZFloat ImuManager::quatToEuler321(Quaternion q) {
    XYZFloat result;
    result.x = atan2f(2*(q.q0*q.q1 + q.q2*q.q3), 1 - 2*(q.q1*q.q1 + q.q2*q.q2));
    result.y = asinf(2*(q.q0*q.q2 - q.q3*q.q1));
    result.z = atan2f(2*(q.q0*q.q3 + q.q1*q.q2), 1 - 2*(q.q2*q.q2 + q.q3*q.q3));
    return result;
}

XYZFloat ImuManager::applySoftIronOffset(XYZFloat mag) {
    XYZFloat result;
    result.x = softIronOffset.row1.x*mag.x + softIronOffset.row1.y*mag.y + softIronOffset.row1.z*mag.z;
    result.y = softIronOffset.row2.x*mag.x + softIronOffset.row2.y*mag.y + softIronOffset.row2.z*mag.z;
    result.z = softIronOffset.row3.x*mag.x + softIronOffset.row3.y*mag.y + softIronOffset.row3.z*mag.z;
    return result;
}

const float ImuManager::alpha = 0.95f;