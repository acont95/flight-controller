#pragma once
#include <stdio.h>
#include <pico/stdlib.h>
#include <ms5611.hpp>
#include <cmath>
// #include <rcl/rcl.h>
// #include <rclc/rclc.h>
// #include <flight_controller_msgs/msg/altitude_temp_pressure.h>

// #define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
// #define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

struct TempPressureAltitude {
    int32_t temp;
    int32_t pressure;
    int32_t altitude;
};

class BaroManager {
    // Reference: https://www.te.com/commerce/DocumentDelivery/DDEController?Action=showdoc&DocId=Data+Sheet%7FMS5611-01BA03%7FB3%7Fpdf%7FEnglish%7FENG_DS_MS5611-01BA03_B3.pdf%7FCAT-BLPS0036
    // Reference: https://cdn-shop.adafruit.com/datasheets/BST-BMP180-DS000-09.pdf
    public:
        BaroManager(MS5611Barometer& ms5611_barometer);
        TempPressureAltitude getAltTempPressure();
        TEMP_AND_PRESSURE getTempAndPressure();
        float tempPressureToAltitude(TEMP_AND_PRESSURE sensor_data);
        void testPrint();
        void publishAltTempPressure();

    private:
        MS5611Barometer& ms5611_barometer;
        static const uint32_t p0 = 101325;

        // rclc_support_t support;
        // rcl_allocator_t allocator;
        // rcl_node_t node;
        // rcl_publisher_t baroTempPublisher;
};