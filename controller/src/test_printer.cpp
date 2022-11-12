#include <test_printer.h>

TestPrinter::TestPrinter(BaroManager& baro_manager, GPSManager& gps_manager, HCRS04Ultrasonic& ultrasonic_sensor, ImuManager& imu_manager) : baro_manager(baro_manager), gps_manager(gps_manager), ultrasonic_sensor(ultrasonic_sensor), imu_manager(imu_manager){
}

void TestPrinter::print() {
    // serial.printf("\n");
    gps_manager.testPrint();
    // serial.printf("\n");
    // baro_manager.testPrint(serial);
    // serial.printf("\n");
    // ultrasonic_sensor.testPrint(serial);
    // serial.printf("\n");
    // imu_manager.testPrint(serial);
    // imu_manager.updateAttitude();
}