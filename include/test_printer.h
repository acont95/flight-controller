#include <mbed.h>
#include <USBSerial.h>
#include <imu_manager.h>
#include <baro_manager.h>
#include <gps_manager.h>
#include <hcsr04_us.h>

class TestPrinter{
    public:
        TestPrinter(USBSerial& serial, BaroManager& baro_manager);
        void print();

    private:
        USBSerial& serial; 
        // GPSManager& gps_manager; 
        BaroManager& baro_manager;
        // HCRS04Ultrasonic& ultrasonic_sensor; 
        // ImuManager& imu_manager;
};
