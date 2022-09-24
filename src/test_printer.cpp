#include <test_printer.h>

TestPrinter::TestPrinter(USBSerial& serial, BaroManager& baro_manager) : serial(serial), baro_manager(baro_manager){
}

void TestPrinter::print() {
    // gps_manager.testPrint(serial);
    baro_manager.testPrint(serial);
    // ultrasonic_sensor.testPrint(serial);
    // imu_manager.testPrint(serial);
    // serial.printf("TEST\n");
}