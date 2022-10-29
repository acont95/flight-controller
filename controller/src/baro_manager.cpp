#include "baro_manager.h"

static void error_loop() {
    while(1){
        // led = !led;
        delay(100);
    }
}

BaroManager::BaroManager(MS5611Barometer& ms5611_barometer) : ms5611_barometer(ms5611_barometer) {
    ms5611_barometer.setOsr(MS5611_OSR_4096);

    allocator = rcl_get_default_allocator();
    RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
    // create node
    RCCHECK(rclc_node_init_default(&node, "baro_temp_node", "", &support));
    // create publisher
    RCCHECK(rclc_publisher_init_default(
        &baroTempPublisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(flight_controller_msgs, msg, AltitudeTempPressure),
        "/mcu/baro_temp_sensor_data"
        )
    );
}

TempPressureAltitude BaroManager::getAltTempPressure() {
    TempPressureAltitude res;
    TEMP_AND_PRESSURE sensor_data = ms5611_barometer.getTempAndPressure();
    res.temp = sensor_data.temp;
    res.pressure = sensor_data.pressure;
    res.altitude = (int32_t) (tempPressureToAltitude(sensor_data) * 1000);

    return res;
}

TEMP_AND_PRESSURE BaroManager::getTempAndPressure() {
    return ms5611_barometer.getTempAndPressure();
}

float BaroManager::tempPressureToAltitude(TEMP_AND_PRESSURE sensor_data) {
    return 44330 * (1 - powf((float)sensor_data.pressure/p0, 1/5.225f));
}

// void BaroManager::testPrint(USBSerial& serial) {
//     ms5611_barometer.readProm();
//     TempPressureAltitude res = getAltTempPressure();
//     serial.printf("Baro Temperature: %i\n", (int) res.temp);
//     serial.printf("Baro Pressure: %i\n", (int) res.pressure);
//     serial.printf("Baro Altitude: %i\n", (int) res.altitude);
// }

void BaroManager::publishAltTempPressure() {
    TempPressureAltitude tpa = getAltTempPressure();
    flight_controller_msgs__msg__AltitudeTempPressure msg = {
        .altitude = tpa.altitude,
        .temperature = tpa.temp,
        .pressure = tpa.pressure
    };
    rcl_publish(&baroTempPublisher, &msg, NULL);
}