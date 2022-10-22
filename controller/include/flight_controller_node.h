#include <mbed.h>
#include <micro_ros_platformio.h>
#include <flight_controller_msgs/msg/telemetry.h>
#include <flight_controller_msgs/msg/controller_command.h>
#include <flight_controller_msgs/msg/gps_coordinates.h>
#include <flight_controller_msgs/msg/height_above_ground.h>
#include <flight_controller_msgs/msg/imu_attitude.h>
#include <flight_controller_msgs/msg/altitude_temp_pressure.h>

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rcl/error_handling.h>

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

mbed::DigitalOut led(LED1);

class FlightControllerNode{
    public:
        FlightControllerNode();
        void publishTelemetry(flight_controller_msgs__msg__Telemetry msg);
        void registerCallbacks(
            rclc_subscription_callback_t imuSensorCallback,
            rclc_subscription_callback_t gpsSensorCallback,
            rclc_subscription_callback_t barometerSensorCallback,
            rclc_subscription_callback_t heightSensorCallback
        );
    private:
        rcl_publisher_t telemtryPublisher;
        rcl_subscription_t imuSensorSubscriber;
        rcl_subscription_t gpsSensorSubscriber;
        rcl_subscription_t barometerSensorSubscriber;
        rcl_subscription_t heightSensorSubscriber;
        rclc_executor_t executor;
        rclc_support_t support;
        rcl_allocator_t allocator;
        rcl_node_t node;

        flight_controller_msgs__msg__Telemetry telemetryMsg;
        flight_controller_msgs__msg__IMUAttitude imuAttitudeMsg;
        flight_controller_msgs__msg__ControllerCommand tcontrollerCommandMsg;
        flight_controller_msgs__msg__GPSCoordinates gpsCoordinatesMsg;
        flight_controller_msgs__msg__AltitudeTempPressure altitudeMsg;
        flight_controller_msgs__msg__HeightAboveGround heightAboveGroundMsg;
				
};
