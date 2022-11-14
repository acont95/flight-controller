#pragma once

#include <nmea_parser.hpp>
// #include <rcl/rcl.h>
// #include <rclc/rclc.h>
// #include <sensor_msgs/msg/nav_sat_fix.h>
// #include <sensor_msgs/msg/nav_sat_status.h>


// #define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
// #define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

// struct GCSCoordinates {
//     double latitude;
//     double longitude;
// };

class GPSManager{
    public:
        GPSManager(); 
        void testPrint();
        void publishGPSCoordinates();
        void setLocation(Location loc);
        Location getLocation();

    private:
        Location loc;
        // int64_t rawToInt(RawDegrees raw);
        
        // rclc_support_t support;
        // rcl_allocator_t allocator;
        // rcl_node_t node;
        // rcl_publisher_t gpsCoordPublisher;
};
