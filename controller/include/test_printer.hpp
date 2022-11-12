#pragma once

#include <imu_manager.h>
#include <baro_manager.h>
#include <gps_manager.h>
#include <hcsr04_us.h>

class TestPrinter{
    public:
        TestPrinter(baroManager& baro_manager, GPSManager& gps_manager, HCRS04Ultrasonic& ultrasonic_sensor, ImuManager& imu_manager);
        void print();

    private:
        BaroManager& baro_manager;
        GPSManager& gps_manager; 
        HCRS04Ultrasonic& ultrasonic_sensor; 
        ImuManager& imu_manager;
};
