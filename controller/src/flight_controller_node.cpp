#include <flight_controller_node.h>

static void error_loop() {
    while(1){
        // led = !led;
        delay(100);
    }
}

FlightControllerNode::FlightControllerNode() {
    allocator = rcl_get_default_allocator();
    RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
    // create node
    RCCHECK(rclc_node_init_default(&node, "flight_controller_node", "", &support));

    // create executor
    RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));

    // create publisher
    RCCHECK(rclc_publisher_init_default(
        &telemtryPublisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(flight_controller_msgs, msg, Telemetry),
        "/telemetry"
        )
    );

    rclc_subscription_init_best_effort(
        &imuSensorSubscriber, 
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(flight_controller_msgs, msg, IMUAttitude), 
        "/mcu/imu_sensor_data");

    rclc_subscription_init_best_effort(
        &gpsSensorSubscriber, 
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(flight_controller_msgs, msg, GPSCoordinates), 
        "/mcu/gps_sensor_data");

    rclc_subscription_init_best_effort(
        &barometerSensorSubscriber, 
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(flight_controller_msgs, msg, AltitudeTempPressure), 
        "/mcu/baro_temp_sensor_data");

    rclc_subscription_init_best_effort(
        &heightSensorSubscriber, 
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(flight_controller_msgs, msg, HeightAboveGround), 
        "/mcu/height_sensor_data");
}

void FlightControllerNode::registerCallbacks(
    rclc_subscription_callback_t imuSensorCallback,
    rclc_subscription_callback_t gpsSensorCallback,
    rclc_subscription_callback_t barometerSensorCallback,
    rclc_subscription_callback_t heightSensorCallback
) {
    rclc_executor_add_subscription(
        &executor, 
        &imuSensorSubscriber, 
        &imuAttitudeMsg,
        imuSensorCallback,
        ON_NEW_DATA);

    rclc_executor_add_subscription(
        &executor, 
        &gpsSensorSubscriber, 
        &gpsCoordinatesMsg,
        gpsSensorCallback,
        ON_NEW_DATA);

    rclc_executor_add_subscription(
        &executor, 
        &barometerSensorSubscriber, 
        &altitudeMsg,
        barometerSensorCallback,
        ON_NEW_DATA);

    rclc_executor_add_subscription(
        &executor, 
        &imuSensorSubscriber, 
        &imuAttitudeMsg,
        imuSensorCallback,
        ON_NEW_DATA);
}

void FlightControllerNode::publishTelemetry(flight_controller_msgs__msg__Telemetry msg) {
    RCSOFTCHECK(rcl_publish(&telemtryPublisher, &msg, NULL));
}