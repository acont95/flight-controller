#include <gps_manager.h>

GPSManager::GPSManager(mbed::UnbufferedSerial& serial_connection, TinyGPSPlus& gps_parser) : serial_connection(serial_connection), gps_parser(gps_parser) {
    serial_connection.baud(9600);
    serial_connection.format(
        /* bits */ 8,
        /* parity */ mbed::SerialBase::None,
        /* stop bit */ 1
    );
}

void GPSManager::updateParser(char data) {
    gps_parser.encode(data);
}

void GPSManager::readData() {
    if (serial_connection.read(&c, 1)) {
        updateParser(c);
    }
}

TinyGPSPlus& GPSManager::getGPS() {
    return gps_parser;
}

void GPSManager::callback() {
    while (true) {
        readData();
        // if (getGPS().location.isUpdated()) {
        //     serial.printf("Latitude: %lf\n", getGPS().location.lat());
        //     serial.printf("Longitude: %lf\n", getGPS().location.lng());
        // }
    }
}