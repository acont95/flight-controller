#pragma once

#include "baro_manager.h"
#include "gps_manager.h"
#include "imu_manager.h"
#include <hcsr04_us.h>

struct ControllerVariables {
    Attitude attitude;
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
    Gain altitude_gain;
};

class FlightController {
    public:
        FlightController(ImuManager& imu_manager, BaroManager& baro_manager, GPSManager& gps_manager, HCRS04Ultrasonic& ultrasonic_sensor);
        void updateMotorOutputs();
        void updateImuAttitude(Attitude attitude);
        void updateImuData();
        void updateBaroData();
        void updateGPSData();
        void updateUltrasonicData();
        void updateGPSAltitude(int32_t altitude_cm);
        void pidUpdate();
        void readSensors();

    private:
        TEMP_PRESSURE_ALTITUDE baro_data;
        const GCSCoordinates home_position;
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
        std::chrono::milliseconds pidUpdatePeriod;
        void calculateControlSignal(Gain gain, ControllerVariables error, ControllerVariables previous_error);
};