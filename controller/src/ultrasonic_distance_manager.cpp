#include <ultrasonic_distance_manager.hpp>

// static void ultrasonic_error_loop() {
//     const uint LED_PIN = PICO_DEFAULT_LED_PIN;
//     gpio_init(LED_PIN);
//     gpio_set_dir(LED_PIN, GPIO_OUT);
//     while(1){
//         gpio_put(LED_PIN, true);
//         sleep_ms(200);
//         gpio_put(LED_PIN, false);
//         sleep_ms(200);
//     }
// }

UltrasonicDistanceManager::UltrasonicDistanceManager(HCRS04Ultrasonic& ultrasonic_sensor) : ultrasonic_sensor(ultrasonic_sensor) {
    // allocator = rcl_get_default_allocator();
    // RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
    // create node
    // RCCHECK(rclc_node_init_default(&node, "baro_temp_node", "", &support));
    // create publisher
    // RCCHECK(rclc_publisher_init_default(
    //     &heightPublisher,
    //     &node,
    //     ROSIDL_GET_MSG_TYPE_SUPPORT(flight_controller_msgs, msg, HeightAboveGround),
    //     "/mcu/height_sensor_data"
    //     )
    // );
}

void UltrasonicDistanceManager::publishHeight() {
    // if (ultrasonic_sensor.shouldTrigger()) {
    //     ultrasonic_sensor.triggerPulse();
    // } 
    // if (ultrasonic_sensor.readReady()) {
    //     uint16_t d_mm = ultrasonic_sensor.getDistanceMm();

    //     flight_controller_msgs__msg__HeightAboveGround msg = {
    //         .height = d_mm
    //     };
    //     rcl_publish(&heightPublisher, &msg, NULL);
    // }
}

void UltrasonicDistanceManager::testPrint() {

    // printf("Should trigger: %s\n", ultrasonic_sensor.shouldTrigger()?"true":"false");
    // printf("Read ready: %s\n", ultrasonic_sensor.readReady()?"true":"false");

    // if (ultrasonic_sensor.shouldTrigger()) {
    //     printf("triggered\n");
    //     ultrasonic_sensor.triggerPulse();
    // } 
    // if (ultrasonic_sensor.readReady()) {
    //     uint32_t d_mm = ultrasonic_sensor.getDistanceMm();
    //     printf("Ultrasonic Sensor (mm): %i \n", (int)d_mm);
    // }
    ultrasonic_sensor.triggerPulse();
    uint32_t d_mm = ultrasonic_sensor.getDistanceMm();
    printf("Ultrasonic Sensor (mm): %i \n", (int)d_mm);

}