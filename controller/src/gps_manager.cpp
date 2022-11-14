#include <gps_manager.hpp>

// static void error_loop() {
//     while(1){
//         // led = !led;
//         delay(100);
//     }
// }

GPSManager::GPSManager() {

    // allocator = rcl_get_default_allocator();
    // RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
    // // create node
    // RCCHECK(rclc_node_init_default(&node, "gps_node", "", &support));
    // // create publisher
    // RCCHECK(rclc_publisher_init_default(
    //     &gpsCoordPublisher,
    //     &node,
    //     ROSIDL_GET_MSG_TYPE_SUPPORT(flight_controller_msgs, msg, GPSCoordinates),
    //     "/mcu/gps_sensor_data"
    //     )
    // );
}

void GPSManager::testPrint() {
    printf("Latitude: %lf\n", loc.lat);
    printf("Longitude: %lf\n", loc.lon);
}

void GPSManager::publishGPSCoordinates() {
    // if (getGPS().location.isUpdated()) {
    //     flight_controller_msgs__msg__GPSCoordinates msg = {
    //         .latitude = getGPS().location.lat(),
    //         .longitude = getGPS().location.lng()
    //     };
    //     rcl_publish(&gpsCoordPublisher, &msg, NULL);
    // }
}

Location GPSManager::getLocation() {
    return loc;
}

void GPSManager::setLocation(Location loc) {
    this->loc = loc;
}
