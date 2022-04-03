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
#include <hcsr04_us.h>

events::EventQueue event_queue(32 * EVENTS_EVENT_SIZE);

USBSerial serial;
/************** SPI BUS DEFINE **************/
static const PinName SPI_CLK = p2;
static const PinName SPI_TX = p3;
static const PinName SPI_RX = p4;
mbed::SPI SPI_BUS_1(SPI_TX, SPI_RX, SPI_CLK);

/************** IMU SENSOR DEFINE **************/
static const PinName IMU_SPI_CS = p5;
static mbed::DigitalOut cs(IMU_SPI_CS);
static const PinName IMU_INT_PIN = p10;
mbed::InterruptIn imu_int(IMU_INT_PIN);
static const xyz16Int accelOffset = {.x=0, .y=0, .z=0};
static const xyz16Int gyroOffset = {.x=0, .y=0, .z=0};
ICM20948_IMU imu(SPI_BUS_1, cs, accelOffset, gyroOffset);
ImuManager imu_manager(imu, imu_int, event_queue);

/************** GPS SENSOR DEFINE **************/
static const PinName GPS_SERIAL_TX = p8;
static const PinName GPS_SERIAL_RX = p9;
static mbed::UnbufferedSerial gps_serial_port(GPS_SERIAL_TX, GPS_SERIAL_RX);
rtos::Thread gps_serial_thread;
TinyGPSPlus gps;
GPSManager gps_manager(gps_serial_port, gps);

/************** BAROMETER SENSOR DEFINE **************/

static const PinName BAROMETER_SPI_CS = p7;
static mbed::DigitalOut baro_cs(BAROMETER_SPI_CS);
MS5611Barometer baro(SPI_BUS_1, baro_cs);
BaroManager baro_manager(baro);

/************** ULTRASONIC SENSOR DEFINE **************/

static const PinName ULTRASONIC_ECHO_PIN = p18;
static const PinName ULTRASONIC_TRIGGER_PIN = p19;
static mbed::DigitalOut ultrasonic_trigger(ULTRASONIC_TRIGGER_PIN);
HCRS04Ultrasonic ultrasonic_sensor(ULTRASONIC_ECHO_PIN, ultrasonic_trigger);

/************** PERIPHERAL COMPUTER DEFINE **************/

static const PinName PI4_SERIAL_TX = p14;
static const PinName PI4_SERIAL_RX = p15;
uint8_t serial_buf[32] = {0};
rtos::Thread peripheral_computer_thread;
// static mbed::BufferedSerial serial_port(SERIAL_TX, SERIAL_RX);


/************** MOTOR DEFINE **************/

// static const mbed::PwmOut motor1(p15);
// static const mbed::PwmOut motor2(p16);
// static const mbed::PwmOut motor3(p17);
// static const mbed::PwmOut motor4(p18);



mbed::DigitalOut led(LED1);
rtos::Thread event_queue_thread;
static const std::chrono::milliseconds pidUpdatePeriod(250);


FlightController fc(imu_manager, baro_manager, gps_manager, ultrasonic_sensor);


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


int main() {

    init();
    baro_cs = 1;

    gps_serial_thread.start(mbed::callback(&gps_manager, &GPSManager::callback));
    // peripheral_computer_thread.start(serialComThread);

    event_queue_thread.start(mbed::callback(&event_queue, &events::EventQueue::dispatch_forever));

    event_queue.call_every(pidUpdatePeriod, mbed::callback(&fc, &FlightController::readSensors));
    event_queue.call_every(pidUpdatePeriod, mbed::callback(&fc, &FlightController::pidUpdate));

}