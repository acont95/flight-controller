#pragma once

#include <baro_manager.h>
#include <gps_manager.h>
#include <imu_manager.h>
#include <hcsr04_us.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>

#include <flight_controller_msgs/msg/telemetry.h>

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

struct ControllerVariables {
    Attitude attitude;
    uint8_t thrust;
    int32_t altitude;
    GCSCoordinates location;
    uint16_t height_above_ground;
};

struct Gain {
    int32_t Kp;
    int32_t Ki;
    int32_t Kd;
};

struct VariableGains {
    Gain roll_gain;
    Gain pitch_gain;
    Gain yaw_gain;
    Gain thrust_gain;
    Gain location_gain;
};

class FlightController {
    public:
        FlightController(ImuManager& imu_manager, BaroManager& baro_manager, GPSManager& gps_manager, HCRS04Ultrasonic& ultrasonic_sensor, GCSCoordinates home_position);
        void updateMotorOutputs();
        void updateImuAttitude(Attitude attitude);
        void updateBaroData(TempPressureAltitude tpa);
        void updateGPSData(GCSCoordinates coordinates);
        void updateHeight(int32_t height_mm);
        void updateGPSAltitude(int32_t altitude_cm);
        void pidUpdate();
        void testPrint(USBSerial& serial);


    private:
        TempPressureAltitude baro_data;
        ControllerVariables system_state;
        ControllerVariables set_point;
        ControllerVariables error;
        ControllerVariables previous_error;
        ControllerVariables control_signal;
        VariableGains gains;
        ImuManager& imu_manager;
        BaroManager& baro_manager;
        GPSManager& gps_manager;
        HCRS04Ultrasonic& ultrasonic_sensor;
        const GCSCoordinates home_position;
        std::chrono::milliseconds pidUpdatePeriod;
        int32_t calculateControlSignal(Gain gain, int32_t error, int32_t previous_error, uint64_t dt);
        mbed::Timer timer;
        void setDt(uint64_t dt);
        uint64_t sensor_read_dt;

        rclc_support_t support;
        rcl_allocator_t allocator;
        rcl_node_t node;
};