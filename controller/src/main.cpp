#include <stdio.h>
#include <pico/stdlib.h>
#include <pico/time.h>
#include <hardware/spi.h>
#include <hardware/uart.h>
#include <inttypes.h>

#include <FreeRTOS.h>
#include <queue.h>
#include <icm20948_imu.hpp>
// #include <flight_controller.h>
#include <imu_manager.hpp>
#include <baro_manager.hpp>
// #include <gps_manager.h>
#include <hcsr04_us.hpp>
#include <ultrasonic_distance_manager.hpp>
// #include <test_printer.h>
// #include <flight_controller_node.h>
#include <pico_gpio_output_impl.hpp>
#include <pico_timer_impl.hpp>
#include <pico_sleep_impl.hpp>
#include <pico_spi_bus_impl.hpp>
#include <nmea_parser.hpp>

/************** FREERTOS MESSAGE QUEUE **************/

static const int msg_queue_len = 128;     // Size of msg_queue
static QueueHandle_t msg_queue;

static PicoSleepImpl sleeper;
static PicoTimer timer;

/************** SPI BUS DEFINE **************/
static const uint SPI_CLK = 2;
static const uint SPI_TX = 3;
static const uint SPI_RX = 4;
static PicoSPIBus SPI_BUS_0(spi0, 2000000);

/************** IMU SENSOR DEFINE **************/
static const uint IMU_SPI_CS = 6;
static PicoGPIOOutputImpl imu_cs(IMU_SPI_CS);
static const uint IMU_INT_PIN = 14;
static const xyz16Int accelOffset = {.x=0, .y=0, .z=0};
static const xyz16Int gyroOffset = {.x=88, .y=-20, .z=10};
static ICM20948_IMU imu(SPI_BUS_0, imu_cs, accelOffset, gyroOffset);
static ImuManager imu_manager(imu, sleeper, timer);

/************** GPS SENSOR DEFINE **************/
static const uint GPS_SERIAL_TX = 16;
static const uint GPS_SERIAL_RX = 17;
static NMEAParser gps;
// static mbed::UnbufferedSerial gps_serial_port(GPS_SERIAL_TX, GPS_SERIAL_RX);
// static rtos::Thread gps_serial_thread;
// static TinyGPSPlus gps;
// static GPSManager gps_manager(uart0, gps);

/************** BAROMETER SENSOR DEFINE **************/
static const uint BAROMETER_SPI_CS = 7;
static PicoGPIOOutputImpl baro_cs(BAROMETER_SPI_CS);
MS5611Barometer baro(SPI_BUS_0, baro_cs, sleeper);
BaroManager baro_manager(baro);

/************** ULTRASONIC SENSOR DEFINE **************/

static const uint ULTRASONIC_ECHO_PIN = 27;
static const uint ULTRASONIC_TRIGGER_PIN = 26;
static const uint32_t ULTRASONIC_EVENT_MASK = GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL;
static PicoGPIOOutputImpl ultrasonic_trigger(ULTRASONIC_TRIGGER_PIN);

static HCRS04Ultrasonic ultrasonic_sensor(ultrasonic_trigger, timer, sleeper);
static UltrasonicDistanceManager distance_manager(ultrasonic_sensor);

// Task: command line interface (CLI)
void imuInterrupt_task(void* parameters) {
    uint8_t recv;
    printf("INTERRUPT TASK\n");
    while (true) {
        if (xQueueReceive(msg_queue, &recv, portMAX_DELAY)) {
            // printf("MESSAGE RECIEVED\n");
            imu_manager.interruptHandler();
        }
    }
}

void imu_irq(void) {
    uint32_t event_mask = gpio_get_irq_event_mask(IMU_INT_PIN);
    if (event_mask == GPIO_IRQ_EDGE_RISE) {
        gpio_acknowledge_irq(IMU_INT_PIN, event_mask);
        BaseType_t xHigherPriorityTaskWoken;
        xHigherPriorityTaskWoken = pdFALSE;
        uint8_t msg = 1;
        xQueueSendFromISR(msg_queue, &msg, NULL);
    } else {
        gpio_acknowledge_irq(IMU_INT_PIN, event_mask);
    }
}


int count = 0;
void ultrasonic_irq(void) {
    count++;
    uint32_t event_mask = gpio_get_irq_event_mask(ULTRASONIC_ECHO_PIN);
    switch(event_mask) {
        case GPIO_IRQ_EDGE_RISE:
            gpio_acknowledge_irq(ULTRASONIC_ECHO_PIN, event_mask);
            ultrasonic_sensor.echoRise();
            break;
        case GPIO_IRQ_EDGE_FALL:
            gpio_acknowledge_irq(ULTRASONIC_ECHO_PIN, event_mask);
            ultrasonic_sensor.echoFall();
            break;
        default:
            gpio_acknowledge_irq(ULTRASONIC_ECHO_PIN, event_mask);
    }
}

// static TestPrinter test_printer(serial, baro_manager, gps_manager, ultrasonic_sensor, imu_manager);

/************** MOTOR DEFINE **************/

// const mbed::PwmOut motor1(p15);
// const mbed::PwmOut motor2(p16);
// const mbed::PwmOut motor3(p17);
// const mbed::PwmOut motor4(p18);

// mbed::DigitalOut led(LED1);
const std::chrono::milliseconds pidUpdatePeriod(50);

// FlightController fc(imu_manager, baro_manager, gps_manager, ultrasonic_sensor, gps_manager.getLocation());


// static void imuSensorCallback(const void* msgin) {
//     const flight_controller_msgs__msg__IMUAttitude* msg = (const flight_controller_msgs__msg__IMUAttitude*) msgin;
//     Attitude a = {
//         .yaw = msg->yaw,
//         .pitch = msg->pitch,
//         .roll = msg->roll
//     };
//     fc.updateImuAttitude(a);
// }

// static void gpsSensorCallback(const void* msgin) {
//     const flight_controller_msgs__msg__GPSCoordinates* msg = (const flight_controller_msgs__msg__GPSCoordinates*) msgin;
//     GCSCoordinates c = {
//         .latitude = msg->latitude,
//         .longitude = msg->longitude
//     };
//     fc.updateGPSData(c);
// }

// static void barometerSensorCallback(const void* msgin) {
//     const flight_controller_msgs__msg__AltitudeTempPressure* msg = (const flight_controller_msgs__msg__AltitudeTempPressure*) msgin;
//     TempPressureAltitude tpa = {
//         .temp = msg->temperature,
//         .pressure = msg->pressure,
//         .altitude = msg->altitude
//     };
//     fc.updateBaroData(tpa);
// }

// static void heightSensorCallback(const void* msgin) {
//     const flight_controller_msgs__msg__HeightAboveGround* msg = (const flight_controller_msgs__msg__HeightAboveGround*) msgin;
//     fc.updateHeight(msg->height);
// }

int main() {
    // msg_queue = xQueueCreate(msg_queue_len, 1);

    stdio_init_all();
    // sleep_ms(5000);
    // // IMU INIT
    // imu_cs.init();
    // imu_cs.setHigh();
    // gpio_init(IMU_INT_PIN);
    // gpio_set_dir(IMU_INT_PIN, false);
    // gpio_set_irq_enabled(IMU_INT_PIN, GPIO_IRQ_EDGE_RISE, true);
    // gpio_add_raw_irq_handler(IMU_INT_PIN, imu_irq);

    // // BARO INIT
    // baro_cs.init();
    // baro_cs.setHigh();

    // // ULTRASONIC SENSOR INIT
    // ultrasonic_trigger.init();
    // gpio_init(ULTRASONIC_ECHO_PIN);
    // gpio_set_dir(ULTRASONIC_ECHO_PIN, false);
    // gpio_set_irq_enabled(ULTRASONIC_ECHO_PIN, ULTRASONIC_EVENT_MASK, true);
    // gpio_add_raw_irq_handler(ULTRASONIC_ECHO_PIN, ultrasonic_irq);

    // irq_set_enabled(IO_IRQ_BANK0, true);

    // // SPI INIT
    // SPI_BUS_0.init();
    // gpio_set_function(SPI_CLK, GPIO_FUNC_SPI);
    // gpio_set_function(SPI_TX, GPIO_FUNC_SPI);
    // gpio_set_function(SPI_RX, GPIO_FUNC_SPI);

    // baro.init();
    // imu_manager.init();

    // BaseType_t imu_task = xTaskCreate(
    //     imuInterrupt_task,
    //     "IMU_INTERRUPT_HANDLER",
    //     1024,
    //     ( void * ) 1,
    //     1,
    //     NULL
    // );

    // if( imu_task == pdPASS )
    // {
    //     /* The task was created.  Use the task's handle to delete the task. */
    //     printf("TASK CREATED\n");
    // } else {
    //     printf("TASK NOT CREATED\n");
    // }

    // vTaskStartScheduler();

    uart_init(uart0, 9600);
    gpio_set_function(GPS_SERIAL_TX, GPIO_FUNC_UART);
    gpio_set_function(GPS_SERIAL_RX, GPIO_FUNC_UART);

    sleeper.sleepMs(4000);
    while (1)
    {
        char c = uart_getc(uart0);
        gps.encode(c);
        if (gps.locationUpdated()) {
            Location loc = gps.getLocation();
            printf("Latitude: %f\n", loc.lat);
            printf("Longitude: %f\n", loc.lon);
        }
        // printf("Count: %i\n", count);
        // distance_manager.testPrint();
        // baro_manager.testPrint();
        // imu_manager.testPrint();
        // printf("char: %c\n", c);
        // sleeper.sleepMs(10);
    }

    // FlightControllerNode fc_node;
    // fc_node.registerCallbacks(
    //     imuSensorCallback,
    //     gpsSensorCallback,
    //     barometerSensorCallback,
    //     heightSensorCallback
    // );
}