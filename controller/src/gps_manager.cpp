#include <gps_manager.h>

static void error_loop() {
    while(1){
        // led = !led;
        delay(100);
    }
}

GPSManager::GPSManager(uart_inst_t& serial_connection, TinyGPSPlus& gps_parser) : serial_connection(serial_connection), gps_parser(gps_parser) {
    uart_init(&serial_connection, 9600);
    uart_set_format(&serial_connection, 8, 1, UART_PARITY_NONE);

    allocator = rcl_get_default_allocator();
    RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
    // create node
    RCCHECK(rclc_node_init_default(&node, "gps_node", "", &support));
    // create publisher
    RCCHECK(rclc_publisher_init_default(
        &gpsCoordPublisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(flight_controller_msgs, msg, GPSCoordinates),
        "/mcu/gps_sensor_data"
        )
    );
}

void GPSManager::updateParser(char data) {
    gps_parser.encode(data);
}

void GPSManager::readData() {
    updateParser(uart_getc(&serial_connection));
}

TinyGPSPlus& GPSManager::getGPS() {
    return gps_parser;
}

void GPSManager::readLoop() {
    while (true) {
        readData();
    }
}

void GPSManager::testPrint() {
    if (getGPS().location.isUpdated()) {
        printf("Latitude: %lf\n", getGPS().location.lat());
        printf("Longitude: %lf\n", getGPS().location.lng());
    }
}

int64_t GPSManager::rawToInt(RawDegrees raw) {
    int64_t num = (int64_t) raw.deg * 1000000000 + raw.billionths;
    if (raw.negative) {
        num = -num;
    }
    return num;
}

GCSCoordinates GPSManager::getLocation() {
    GCSCoordinates loc;
    loc.latitude = rawToInt(gps_parser.location.rawLat());
    loc.longitude = rawToInt(gps_parser.location.rawLng());
    return loc;
}

void GPSManager::publishGPSCoordinates() {
    if (getGPS().location.isUpdated()) {
        flight_controller_msgs__msg__GPSCoordinates msg = {
            .latitude = getGPS().location.lat(),
            .longitude = getGPS().location.lng()
        };
        rcl_publish(&gpsCoordPublisher, &msg, NULL);
    }
}