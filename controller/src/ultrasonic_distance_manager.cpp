#include <ultrasonic_distance_manager.h>

static void error_loop() {
    while(1){
        // led = !led;
        delay(100);
    }
}

UltrasonicDistanceManager::UltrasonicDistanceManager(HCRS04Ultrasonic& ultrasonic_sensor) : ultrasonic_sensor(ultrasonic_sensor) {
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

UltrasonicDistanceManager::publishHeight() {
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