#include <mbed.h>
#include <cstdint>
#include <icm20948_imu.h>
#include <message_protocol.h>
#include <flight_controller.h>
#include <imu_manager.h>
#include <baro_manager.h>
#include <stdio.h>
#include <USBSerial.h>
#include <TinyGPS++.h>
#include <gps_manager.h>
#include <hcsr04_us.h>
#include <test_printer.h>

static USBSerial serial;
static events::EventQueue event_queue(32 * EVENTS_EVENT_SIZE);
static rtos::Thread event_queue_thread;
static rtos::Thread imu_fifo_poll_thread;

/************** SPI BUS DEFINE **************/
static const PinName SPI_CLK = p2;
static const PinName SPI_TX = p3;
static const PinName SPI_RX = p4;
static mbed::SPI SPI_BUS_1(SPI_TX, SPI_RX, SPI_CLK);

static const PinName IMU_SPI_CS = p6;
static mbed::DigitalOut cs(IMU_SPI_CS, 1);

static const PinName BAROMETER_SPI_CS = p7;
static mbed::DigitalOut baro_cs(BAROMETER_SPI_CS, 1);

/************** IMU SENSOR DEFINE **************/
static const PinName IMU_INT_PIN = p14;
static mbed::InterruptIn imu_int(IMU_INT_PIN);
static const xyz16Int accelOffset = {.x=0, .y=0, .z=0};
// static const xyz16Int gyroOffset = {.x=0, .y=0, .z=0};
static const xyz16Int gyroOffset = {.x=88, .y=-20, .z=10};

static ICM20948_IMU imu(SPI_BUS_1, cs, accelOffset, gyroOffset, serial);
static ImuManager imu_manager(imu, imu_int, event_queue, serial);

/************** GPS SENSOR DEFINE **************/
static const PinName GPS_SERIAL_TX = p16;
static const PinName GPS_SERIAL_RX = p17;
static mbed::UnbufferedSerial gps_serial_port(GPS_SERIAL_TX, GPS_SERIAL_RX);
static rtos::Thread gps_serial_thread;
static TinyGPSPlus gps;
static GPSManager gps_manager(gps_serial_port, gps);

/************** BAROMETER SENSOR DEFINE **************/
MS5611Barometer baro(SPI_BUS_1, baro_cs);
BaroManager baro_manager(baro);

/************** ULTRASONIC SENSOR DEFINE **************/

static const PinName ULTRASONIC_ECHO_PIN = p27;
static const PinName ULTRASONIC_TRIGGER_PIN = p26;
static mbed::DigitalOut ultrasonic_trigger(ULTRASONIC_TRIGGER_PIN);
static HCRS04Ultrasonic ultrasonic_sensor(ULTRASONIC_ECHO_PIN, ultrasonic_trigger);

static TestPrinter test_printer(serial, baro_manager, gps_manager, ultrasonic_sensor, imu_manager);


/************** PERIPHERAL COMPUTER DEFINE **************/

// uint8_t serial_buf[32] = {0};
// rtos::Thread peripheral_computer_thread;
// mbed::UnbufferedSerial pc_serial_port(USBTX, USBRX);

/************** MOTOR DEFINE **************/

// const mbed::PwmOut motor1(p15);
// const mbed::PwmOut motor2(p16);
// const mbed::PwmOut motor3(p17);
// const mbed::PwmOut motor4(p18);

mbed::DigitalOut led(LED1);
const std::chrono::milliseconds pidUpdatePeriod(10);

// FlightController fc(imu_manager, baro_manager, gps_manager, ultrasonic_sensor, gps_manager.getLocation());

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

    gps_serial_thread.start(mbed::callback(&gps_manager, &GPSManager::callback));
    gps_serial_thread.set_priority(osPriorityNormal);

    // peripheral_computer_thread.start(serialComThread);
    event_queue_thread.start(mbed::callback(&event_queue, &events::EventQueue::dispatch_forever));
    event_queue_thread.set_priority(osPriorityHigh);
    // imu_fifo_poll_thread.start(mbed::callback(&imu_manager, &ImuManager::pollFifo));
    // event_queue.call_every(pidUpdatePeriod, mbed::callback(&fc, &FlightController::readSensors));
    // event_queue.call_every(pidUpdatePeriod, mbed::callback(&fc, &FlightController::pidUpdate));
    event_queue.call_every(events::EventQueue::duration(1000), &test_printer, &TestPrinter::print);
}