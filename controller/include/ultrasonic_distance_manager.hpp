#pragma once
#include <stdio.h>
#include <pico/stdlib.h>
#include <hcsr04_us.hpp>
// #include <rcl/rcl.h>
// #include <rclc/rclc.h>
// #include <flight_controller_msgs/msg/height_above_ground.h>

// static void ultrasonic_error_loop();
// #define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){ultrasonic_error_loop();}}
// #define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

class UltrasonicDistanceManager{
    public:
        UltrasonicDistanceManager(HCRS04Ultrasonic& ultrasonic_sensor); 
        void publishHeight();
        void testPrint();

    private:
        HCRS04Ultrasonic& ultrasonic_sensor;
        // rclc_support_t support;
        // rcl_allocator_t allocator;
        // rcl_node_t node;
        // rcl_publisher_t heightPublisher;
};
