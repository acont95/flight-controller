#include "hcsr04_us.h"

static void error_loop() {
    while(1){
        // led = !led;
        delay(100);
    }
}

HCRS04Ultrasonic::HCRS04Ultrasonic(const PinName& echo_pin, mbed::DigitalOut& trigger): echo_int(echo_pin), trigger(trigger) {
    echo_int.rise(mbed::callback(this, &HCRS04Ultrasonic::echoRise));
    echo_int.fall(mbed::callback(this, &HCRS04Ultrasonic::echoFall));

    allocator = rcl_get_default_allocator();
    RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
    // create node
    RCCHECK(rclc_node_init_default(&node, "baro_temp_node", "", &support));
    // create publisher
    RCCHECK(rclc_publisher_init_default(
        &heightPublisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(flight_controller_msgs, msg, HeightAboveGround),
        "/mcu/height_sensor_data"
        )
    );
}

void HCRS04Ultrasonic::triggerPulse() {
    trigger=1;
    wait_us(10);
    trigger=0;
    updated = false;
    triggered = true;
}

void HCRS04Ultrasonic::echoRise() {
    timer.start();
}

void HCRS04Ultrasonic::echoFall() {
    timer.stop();
    t = timer.elapsed_time();
    timer.reset();
    updated = true;
    triggered = false;
}

uint32_t HCRS04Ultrasonic::getDistanceMm() {
    updated = false;
    return (t.count() * c_nm_us_half) / 1000000;
}

bool HCRS04Ultrasonic::isUpdated() {
    return updated;
}

bool HCRS04Ultrasonic::isTriggered() {
    return triggered;
}

bool HCRS04Ultrasonic::shouldTrigger() {
    return !isTriggered() && !isUpdated();
}

bool HCRS04Ultrasonic::readReady() {
    return !isTriggered() && isUpdated();
}

void HCRS04Ultrasonic::testPrint(USBSerial& serial) {
    if (shouldTrigger()) {
        triggerPulse();
    } 
    if (readReady()) {
        uint32_t d_mm = getDistanceMm();
        serial.printf("Ultrasonic Sensor (mm): %i \n", (int)d_mm);
    }
}

void HCRS04Ultrasonic::publishHeight() {
    if (shouldTrigger()) {
        triggerPulse();
    } 
    if (readReady()) {
        uint16_t d_mm = getDistanceMm();

        flight_controller_msgs__msg__HeightAboveGround msg = {
            .height = d_mm
        };
        rcl_publish(&heightPublisher, &msg, NULL);
    }
}