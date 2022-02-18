#include <mbed.h>
#include <cstdint>
#include <icm20948_imu.h>
#include <message_protocol.h>
#include "flight_controller.h";
#include "imu_manager.h"

/************** SPI PIN DEFINE **************/
static const PinName IMU_SPI_CS = p5;
static const PinName GPS_SPI_CS = p6;
static const PinName BAROMETER_SPI_CS = p7;

static const PinName SPI_CLK = p2;
static const PinName SPI_TX = p3;
static const PinName SPI_RX = p4;

static const PinName IMU_INT_PIN = p10;

/************** SERIAL PIN DEFINE **************/
static const PinName SERIAL_TX = p14;
static const PinName SERIAL_RX = p15;
uint8_t serial_buf[32] = {0};

static const mbed::PwmOut motor1(p15);
static const mbed::PwmOut motor2(p16);
static const mbed::PwmOut motor3(p17);
static const mbed::PwmOut motor4(p18);
static mbed::Timer t;

static mbed::DigitalOut cs(IMU_SPI_CS);
static mbed::BufferedSerial serial_port(SERIAL_TX, SERIAL_RX);

static const xyzOffset accelOffset = {.x=0, .y=0, .z=0};
static const xyzOffset gyroOffset = {.x=0, .y=0, .z=0};

mbed::InterruptIn imu_int(p10);
events::EventQueue event_queue(32 * EVENTS_EVENT_SIZE);
rtos::Thread event_queue_thread;
rtos::Thread peripheral_computer_thread;
mbed::SPI SPI_BUS_1(SPI_CLK, SPI_TX, SPI_RX);
ICM20948_IMU imu(SPI_BUS_1, cs, accelOffset, gyroOffset);
FlightController fc = FlightController(GpsCoord{.latitude = 0, .longitude= 0});
ImuManager imu_manager(imu);

void setup(void)
{
    event_queue_thread.set_priority(osPriorityRealtime);
    peripheral_computer_thread.set_priority(osPriorityNormal);

    pinMode(IMU_SPI_CS, OUTPUT);
    cs = 1;
    serial_port.set_baud(9600);
    serial_port.set_format(
        /* bits */ 8,
        /* parity */ mbed::BufferedSerial::None,
        /* stop bit */ 1
    );

    // ICM20948_INT.rise();
}

void serialComThread() {
    uint32_t num;
    MessageProtocol prot = MessageProtocol();
    while (true) {
        num = serial_port.read(serial_buf, sizeof(serial_buf));
        if (num) {
            // if (prot.decodeData(serial_buf, sizeof(serial_buf))) {

            // }
        }
    }
}

void imuSensorIsr() {
    t.stop();
    imu_manager.setDt(t.elapsed_time().count());
    t.reset();
    t.start();
    event_queue.call(flightControllerUpdate);
}

void flightControllerUpdate() {
    // imu.
    // fc.updateYawPitchRoll(0,0,0);
    // fc.updateMotorOutputs();
}

int main() {
    peripheral_computer_thread.start(serialComThread);
    event_queue_thread.start(mbed::callback(&event_queue, &events::EventQueue::dispatch_forever));
    imu_int.rise(imuSensorIsr);
    mktime
    // init();>?
    setup();
}