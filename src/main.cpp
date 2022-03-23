#include <mbed.h>
#include <cstdint>
#include <icm20948_imu.h>
#include <message_protocol.h>
#include "flight_controller.h"
#include "imu_manager.h"
#include "baro_manager.h"
#include <stdio.h>
#include <USBSerial.h>
#include <TinyGPS++.h>
#include <gps_manager.h>


USBSerial serial;
/************** SPI PIN DEFINE **************/
static const PinName IMU_SPI_CS = p5;
static const PinName GPS_SPI_CS = p6;
static const PinName BAROMETER_SPI_CS = p7;

static const PinName SPI_CLK = p2;
static const PinName SPI_TX = p3;
static const PinName SPI_RX = p4;

static const PinName IMU_INT_PIN = p10;

/************** SERIAL PIN DEFINE **************/
static const PinName GPS_SERIAL_TX = p8;
static const PinName GPS_SERIAL_RX = p9;
static const PinName PI4_SERIAL_TX = p14;
static const PinName PI4_SERIAL_RX = p15;
uint8_t serial_buf[32] = {0};

static const mbed::PwmOut motor1(p15);
static const mbed::PwmOut motor2(p16);
static const mbed::PwmOut motor3(p17);
static const mbed::PwmOut motor4(p18);
static mbed::Timer t;

static mbed::DigitalOut cs(IMU_SPI_CS);
static mbed::DigitalOut baro_cs(BAROMETER_SPI_CS);
mbed::DigitalOut led(LED1);


// ERROR
// static mbed::BufferedSerial serial_port(SERIAL_TX, SERIAL_RX);
static mbed::UnbufferedSerial gps_serial_port(GPS_SERIAL_TX, GPS_SERIAL_RX);


static const xyz16Int accelOffset = {.x=0, .y=0, .z=0};
static const xyz16Int gyroOffset = {.x=0, .y=0, .z=0};

mbed::InterruptIn imu_int(p10);
events::EventQueue event_queue(32 * EVENTS_EVENT_SIZE);
rtos::Thread event_queue_thread;
rtos::Thread peripheral_computer_thread;
rtos::Thread gps_serial_thread;
mbed::SPI SPI_BUS_1(SPI_TX, SPI_RX, SPI_CLK);

// ICM20948_IMU imu(SPI_BUS_1, cs, accelOffset, gyroOffset);
// ImuManager imu_manager(imu);
// FlightController fc(GpsCoord{.latitude = 0, .longitude= 0});

MS5611Barometer baro(SPI_BUS_1, baro_cs);
BaroManager baro_manager(baro);
TinyGPSPlus gps;
GPSManager gps_manager(gps_serial_port, gps);

// // static const std::chrono::milliseconds pidUpdatePeriod(10);
static const std::chrono::milliseconds pidUpdatePeriod(5);


void setup(void)
{
    // event_queue_thread.set_priority(osPriorityRealtime);
    // peripheral_computer_thread.set_priority(osPriorityNormal);

    // pinMode(IMU_SPI_CS, OUTPUT);
    // cs = 1;

}

// void serialComThread() {
//     uint32_t num;
//     MessageProtocol prot = MessageProtocol();
//     while (true) {
//         num = serial_port.read(serial_buf, sizeof(serial_buf));
//         if (num) {
//             // if (prot.decodeData(serial_buf, sizeof(serial_buf))) {

//             // }
//         }
//     }
// }

void gpsComThread() {
    while (true) {
        // rtos::ThisThread::sleep_for(rtos::Kernel::Clock::duration_u32 {10});
        gps_manager.readData();
        if (gps_manager.getGPS().location.isUpdated()) {
            serial.printf("Latitude: %lf\n", gps_manager.getGPS().location.lat());
            serial.printf("Longitude: %lf\n", gps_manager.getGPS().location.lng());
        }
    }
}

// void imuSensorIsr() {

//     switch (imu_manager.getInterruptType()) {
//         case DATA_READY:
//             t.stop();
//             imu_manager.setDt(t.elapsed_time().count());
//             t.reset();
//             t.start();
//             event_queue.call(&imu_manager, &ImuManager::updateAttitude);
//             break;
//         case FIFO_OVERFLOW:
//             event_queue.call(&imu_manager, &ImuManager::resetFifo);
//             break;
//         case NO_VALID_INTERRUPT:
//             break;
//     }
//     event_queue.call(&imu_manager, &ImuManager::interruptClear);
// }

void pidUpdate() {
    // fc.updateAttitude(imu_manager.getAttitude());
    TEMP_PRESSURE_ALTITUDE sensor_data = baro_manager.getAltTempPressure();
    // led = !led;

    // serial.printf("altitude float: %i\n", sensor_data.altitude);
    // serial.printf("pressure: %i\n", sensor_data.pressure);
    // serial.printf("temp: %i\n\n", sensor_data.temp);
    // fc.updateTempPressure();
}

int main() {
    // setup();

    init();
    baro_cs = 1;

    gps_serial_thread.start(gpsComThread);
    // peripheral_computer_thread.start(serialComThread);
    // imu_int.rise(imuSensorIsr);

    event_queue_thread.start(mbed::callback(&event_queue, &events::EventQueue::dispatch_forever));
    event_queue.call_every(pidUpdatePeriod, pidUpdate);
}