#pragma once

#include <hcsr04_us.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <flight_controller_msgs/msg/height_above_ground.h>
#include <zephyr/kernel.h>


// struct k_timer my_timer;


#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

class UltrasonicDistanceManager{
    public:
        UltrasonicDistanceManager(HCRS04Ultrasonic& ultrasonic_sensor, k_timer timer); 
        void publishHeight();


    private:
        HCRS04Ultrasonic& ultrasonic_sensor;
        
        rclc_support_t support;
        rcl_allocator_t allocator;
        rcl_node_t node;
        rcl_publisher_t gpsCoordPublisher;
};
