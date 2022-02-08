// #include <Arduino.h>
#include <mbed.h>
#include <cstdint>
// #include <SPI.h>
#include "icm20948_imu.h"

static const PinName SPI_CS = p5;
static const PinName SPI_CLK = p2;
static const PinName SPI_TX = p3;
static const PinName SPI_RX = p4;
static const PinName IMU_INT_PIN = p10;

static const xyzOffset accelOffset = {.x=0, .y=0, .z=0};
static const xyzOffset gyroOffset = {.x=0, .y=0, .z=0};

mbed::InterruptIn ICM20948_INT(p10);
events::EventQueue event_queue(32 * EVENTS_EVENT_SIZE);
// Thread event_queue_thread;
// MbedSPI SPI_BUS_1(SPI_CLK, SPI_TX, SPI_RX);
mbed::SPI SPI_BUS_1(SPI_CLK, SPI_TX, SPI_RX);
// MbedSPI SPI_BUS_2(SPI_CLK, SPI_TX, SPI_RX);
ICM20948_IMU imu(& SPI_BUS_1, SPI_CS, accelOffset, gyroOffset);

void setup(void)
{

    pinMode(SPI_CS, OUTPUT);
    digitalWrite(SPI_CS, HIGH);
    SPI_BUS_1.begin();

    imu.setUserCtrl(DMP_DISABLED, FIFO_ENABLED, I2C_DISABLED, SPI_MODE, DMP_RST_DO_NOTHING, SRAM_RST_DO_NOTHING, I2C_RST_DO_NOTHING);
    imu.setPwrMgmt1(DEVICE_RESET_DO_NOTHING, SLEEP_MODE_DISABLED, LOW_POWER_MODE_DISABLED, TEMP_SENSOR_DISABLED, AUTO_CLK);
    imu.setPwrMgmt2(GYRO_ENABLE, ACCEL_ENABLE);
    imu.setIntPinCfg(INT1_LOGIC_HIGH, INT1_PUSH_PULL, INT1_HELD_UNTIL_CLEARED, INT_STATUS_READ_CLEAR, FSYNC_ACTIVE_HIGH, FSYNC_INT_DISABLE, I2C_MASTER_BYPASS_MODE_DISABLE);
    imu.setIntEnable(DISABLE_WAKE_FSYNC, DISABLE_WAKE_MOTION, DISABLE_PLL_RDY_INT, DISABLE_DMP_INT, DISABLE_I2C_MASTER_INT);
    imu.setIntEnable1(ENABLE_RAW_DATA_INT);
    imu.setFifoEn2(ENABLE_ACCEL_FIFO, ENABLE_GYRO_Z_FIFO, ENABLE_GYRO_Y_FIFO, ENABLE_GYRO_X_FIFO, DISABLE_TEMP_FIFO);
    imu.setFifoMode(FIFO_STREAM);
    imu.setGyroSmplrtDiv(0);
    imu.setGyroConfig1(GYRO_DLPF_0, GYRO_RANGE_1000, GYRO_ENABLE_DLPF);
    imu.setOdrAlignEn(ENABLE_ODR_START_TIME_ALIGNMENT);
    imu.setAccelSmplrtDiv(0);
    imu.setAccelConfig(ACCEL_DLPFCFG_0, ACCEL_FULL_SCALE_16G, ACCEL_ENABLE_DLPF);

    ICM20948_INT.rise();
}

void loop(void)
{
    // SPI_BUS_1.beginTransaction(imu.getSpiSettings());
}

int main() {
    init();
    setup();
}