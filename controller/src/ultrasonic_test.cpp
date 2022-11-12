#include <FreeRTOS.h>
#include <stdio.h>
#include <pico/stdlib.h>
#include <pico/time.h>
#include <pico_gpio_impl.hpp>
#include <pico_timer_impl.hpp>
#include <pico_sleep_impl.hpp>
#include <hcsr04_us.hpp>
#include <ultrasonic_distance_manager.hpp>

/************** ULTRASONIC SENSOR DEFINE **************/

static const uint ULTRASONIC_ECHO_PIN = 27;
static const uint ULTRASONIC_TRIGGER_PIN = 26;

static PicoGPIOImpl ultrasonic_trigger(ULTRASONIC_TRIGGER_PIN);
static PicoTimer ultrasonic_timer;
static PicoSleepImpl ultrasonic_sleeper;

static HCRS04Ultrasonic ultrasonic_sensor(ultrasonic_trigger, ultrasonic_timer, ultrasonic_sleeper);
static UltrasonicDistanceManager distance_manager(ultrasonic_sensor);


void ultrasonic_irq(uint gpio, uint32_t events) {
    // printf("IRQ\n");
    switch(events) {
        case GPIO_IRQ_EDGE_RISE:
            printf("RISING EDGE\n");

            ultrasonic_sensor.echoRise();
            break;
        case GPIO_IRQ_EDGE_FALL:
            printf("FALLING EDGE\n");

            ultrasonic_sensor.echoFall();
            break;
    }
}

int main(void) {
    stdio_init_all();
    sleep_ms(5000);
    gpio_init(ULTRASONIC_ECHO_PIN);
    gpio_set_irq_enabled_with_callback(ULTRASONIC_ECHO_PIN, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true, &ultrasonic_irq);
    while (1)
    {
        // printf("ULTRASONIC TEST\n");
        distance_manager.testPrint();
        // ultrasonic_trigger.setHigh();
        ultrasonic_sleeper.sleepMs(1000);
        // ultrasonic_trigger.setLow();
        // ultrasonic_sleeper.sleepUs(1000000);
    }
}