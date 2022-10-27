#pragma once

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <flight_controller_msgs/msg/gps_coordinates.h>

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

struct GCSCoordinates {
    double latitude;
    double longitude;
};

class GPSManager{
    public:
        GPSManager(mbed::UnbufferedSerial& serial_connection, TinyGPSPlus& gps_parser); 
        void readData();
        // TinyGPSPlus& getGPS();
        void readLoop();
        GCSCoordinates getLocation();
        // void testPrint(USBSerial& serial);
        void publishGPSCoordinates();

    private:
        mbed::UnbufferedSerial& serial_connection;
        TinyGPSPlus& gps_parser;
        uint8_t c;   
        void updateParser(char data);
        int64_t rawToInt(RawDegrees raw);
        
        rclc_support_t support;
        rcl_allocator_t allocator;
        rcl_node_t node;
        rcl_publisher_t gpsCoordPublisher;
};
