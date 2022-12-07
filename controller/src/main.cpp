#include <stdio.h>
#include <pico/stdlib.h>
#include <pico/time.h>
#include <hardware/spi.h>
#include <hardware/uart.h>
#include <inttypes.h>

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rmw_microros/rmw_microros.h>
#include <pico_uart_transports.h>


#include <FreeRTOS.h>
#include <semphr.h>
#include <queue.h>
#include <freertos_sleep_impl.hpp>

#include <icm20948_imu.hpp>
// #include <flight_controller.h>
#include <imu_manager.hpp>
#include <baro_manager.hpp>
#include <gps_manager.hpp>
#include <hcsr04_us.hpp>
#include <ultrasonic_distance_manager.hpp>
// #include <test_printer.h>
#include <flight_controller_node.hpp>
#include <pico_gpio_output_impl.hpp>
#include <pico_timer_impl.hpp>
#include <pico_sleep_impl.hpp>
#include <pico_spi_bus_impl.hpp>
#include <nmea_parser.hpp>
#include <error_loop.h>

const uint LED_PIN = PICO_DEFAULT_LED_PIN;

void error_loop() {
    while(1){
        gpio_put(LED_PIN, !gpio_get(LED_PIN));
        sleep_ms(100);
    }
}

/************** FREERTOS DECLARATIONS **************/

static const int msg_queue_len = 128;  // Size of msg_queue
static QueueHandle_t msg_queue;
static SemaphoreHandle_t spi_mutex;
static FreeRTOSSleepImpl rtosSleeper;

static PicoSleepImpl sleeper;
static PicoTimer timer;

/************** SPI BUS DEFINE **************/
static const uint SPI_CLK = 2;
static const uint SPI_TX = 3;
static const uint SPI_RX = 4;
static PicoSPIBus SPI_BUS_0(spi0, &spi_mutex, 2000000);

/************** IMU SENSOR DEFINE **************/
static const uint IMU_SPI_CS = 6;
static PicoGPIOOutputImpl imu_cs(IMU_SPI_CS);
static const uint IMU_INT_PIN = 14;
static const XYZ16Int accelOffset = {.x=0, .y=0, .z=0};
static const XYZ16Int gyroOffset = {.x=88, .y=-19, .z=9};
static const XYZ16Int hardIronOffset = {.x=-141, .y=75, .z=-82};
static const SoftIronOffset softIronOffset = {
    {.x=1.4475028682818498,.y=-0.0028425603218517267,.z=0.04243430365775245},
    {.x=-0.0028425603218516487,.y=1.447234258663461,.z=0.06844794461759508},
    {.x=0.04243430365775225,.y=0.06844794461759497,.z=0.9914372150252756}
};

static ICM20948_IMU imu(SPI_BUS_0, imu_cs, sleeper, accelOffset, gyroOffset);
static ImuManager imu_manager(imu, sleeper, timer, hardIronOffset, softIronOffset, 12);

/************** GPS SENSOR DEFINE **************/
static const uint GPS_SERIAL_TX = 16;
static const uint GPS_SERIAL_RX = 17;
static NMEAParser gps;
static GPSManager gps_manager;

// Task: Serial UART task
void gps_feed_task(void* parameters) {
    uint8_t recv;
    while (true) {
        char c = uart_getc(uart0);
        gps.encode(c);
        // Push location data when valid 
        if (gps.locationUpdated()) {
            Location loc = gps.getLocation();
            gps_manager.setLocation(loc);
            // gps_manager.publishGPSCoordinates();
        }
    }
}

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

// Task: IMU interrupt
void imuInterrupt_task(void* parameters) {
    uint8_t recv;
    imu_manager.enableInterrupts();
    while (true) {
        if (xQueueReceive(msg_queue, &recv, portMAX_DELAY)) {
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

// Flight Controller ROS Node
// static FlightControllerNode fcNode;


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

void periodic_publish_task(void* parameters) {
    while(true) {
        rtosSleeper.sleepMs(20);
        imu_manager.testPrint();
        // baro_manager.publishAltTempPressure();
    }
}

// static TestPrinter test_printer(serial, baro_manager, gps_manager, ultrasonic_sensor, imu_manager);

/************** MOTOR DEFINE **************/

// const mbed::PwmOut motor1(p15);
// const mbed::PwmOut motor2(p16);
// const mbed::PwmOut motor3(p17);
// const mbed::PwmOut motor4(p18);

const std::chrono::milliseconds pidUpdatePeriod(50);

// FlightController fc(imu_manager, baro_manager, gps_manager, ultrasonic_sensor, gps_manager.getLocation());



int main() {
    msg_queue = xQueueCreate(msg_queue_len, 1);
    spi_mutex = xSemaphoreCreateMutex();

    // rmw_uros_set_custom_transport(
	// 	true,
	// 	NULL,
	// 	pico_serial_transport_open,
	// 	pico_serial_transport_close,
	// 	pico_serial_transport_write,
	// 	pico_serial_transport_read
	// );

    // // Wait for agent successful ping for 2 minutes.
    // const int timeout_ms = 1000; 
    // const uint8_t attempts = 120;

    // rcl_ret_t ret = rmw_uros_ping_agent(timeout_ms, attempts);

    // if (ret != RCL_RET_OK)
    // {
    //     // Unreachable agent, exiting program.
    //     return ret;
    // }

    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, true);

    stdio_init_all();
    sleep_ms(5000);

    // GPS UART INIT
    uart_init(uart0, 9600);
    gpio_set_function(GPS_SERIAL_TX, GPIO_FUNC_UART);
    gpio_set_function(GPS_SERIAL_RX, GPIO_FUNC_UART);

    // BaseType_t gps_task = xTaskCreate(
    //     gps_feed_task,
    //     "GPS_FEED_TASK",
    //     1024,
    //     ( void * ) 1,
    //     1,
    //     NULL
    // );

    // IMU INIT
    imu_cs.init();
    imu_cs.setHigh();
    gpio_init(IMU_INT_PIN);
    gpio_set_dir(IMU_INT_PIN, false);
    gpio_set_irq_enabled(IMU_INT_PIN, GPIO_IRQ_EDGE_RISE, true);
    gpio_add_raw_irq_handler(IMU_INT_PIN, imu_irq);

    BaseType_t imu_task = xTaskCreate(
        imuInterrupt_task,
        "IMU_INTERRUPT_HANDLER",
        1024,
        ( void * ) 1,
        2,
        NULL
    );

    // BARO INIT
    baro_cs.init();
    baro_cs.setHigh();

    // ULTRASONIC SENSOR INIT
    // ultrasonic_trigger.init();
    // gpio_init(ULTRASONIC_ECHO_PIN);
    // gpio_set_dir(ULTRASONIC_ECHO_PIN, false);
    // gpio_set_irq_enabled(ULTRASONIC_ECHO_PIN, ULTRASONIC_EVENT_MASK, true);
    // gpio_add_raw_irq_handler(ULTRASONIC_ECHO_PIN, ultrasonic_irq);

    irq_set_enabled(IO_IRQ_BANK0, true);

    // SPI INIT
    SPI_BUS_0.init();
    gpio_set_function(SPI_CLK, GPIO_FUNC_SPI);
    gpio_set_function(SPI_TX, GPIO_FUNC_SPI);
    gpio_set_function(SPI_RX, GPIO_FUNC_SPI);

    // fcNode.init();
    // baro.init();
    imu_manager.init();
    // gps_manager.init(fcNode.getRosNode());s

    // BaseType_t periodic_pub_task = xTaskCreate(
    //     periodic_publish_task,
    //     "PERIODIC_PUBLISH_TASK",
    //     1024,
    //     ( void * ) 1,
    //     1,
    //     NULL
    // );

    vTaskStartScheduler();

    // FlightControllerNode fc_node;
    // fc_node.registerCallbacks(
    //     imuSensorCallback,
    //     gpsSensorCallback,
    //     barometerSensorCallback,
    //     heightSensorCallback
    // );
}