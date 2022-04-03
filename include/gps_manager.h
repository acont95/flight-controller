#pragma once

#include <TinyGPS++.h>
#include <mbed.h>
#include <USBSerial.h>

struct GCSCoordinates {
    RawDegrees latitude;
    RawDegrees longitude;
};

class GPSManager{
    public:
        GPSManager(mbed::UnbufferedSerial& serial_connection, TinyGPSPlus& gps_parser); 
        void readData();
        TinyGPSPlus& getGPS();
        void callback();

    private:
        mbed::UnbufferedSerial& serial_connection;
        TinyGPSPlus& gps_parser;
        uint8_t c;   
        void updateParser(char data);
};
