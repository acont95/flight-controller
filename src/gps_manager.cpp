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
    }
}

void GPSManager::testPrint(USBSerial& serial) {
    if (getGPS().location.isUpdated()) {
        serial.printf("Latitude: %lf\n", getGPS().location.lat());
        serial.printf("Longitude: %lf\n", getGPS().location.lng());
    }
}

int64_t GPSManager::rawToInt(RawDegrees raw) {
    int64_t num = (int64_t) raw.deg * 1000000000 + raw.billionths;
    if (raw.negative) {
        num = -num;
    }
    return num;
}

GCSCoordinates GPSManager::getLocation() {
    GCSCoordinates loc;
    loc.latitude = rawToInt(gps_parser.location.rawLat());
    loc.longitude = rawToInt(gps_parser.location.rawLng());
    return loc;
}