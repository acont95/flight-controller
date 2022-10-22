#pragma once

#include <mbed.h>
#include <USBSerial.h>
#include <platform/mbed_wait_api.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <flight_controller_msgs/msg/height_above_ground.h>

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

class HCRS04Ultrasonic {
    public:
        HCRS04Ultrasonic(const PinName& echo_pin, mbed::DigitalOut& trigger);
        void triggerPulse();
        uint32_t getDistanceMm();
        bool shouldTrigger();
        bool readReady();
        void testPrint(USBSerial& serial);
        void publishHeight();

    private:
        mbed::InterruptIn echo_int;
        mbed::Timer timer;
        mbed::DigitalOut& trigger;
        void echoRise();
        void echoFall();
        std::chrono::microseconds t;
        uint64_t c_nm_us_half = 171500;
        bool updated = false;
        bool triggered = false;
        bool isUpdated();
        bool isTriggered();

        rclc_support_t support;
        rcl_allocator_t allocator;
        rcl_node_t node;
        rcl_publisher_t heightPublisher;
};