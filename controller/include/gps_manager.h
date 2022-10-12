#pragma once

#include <TinyGPS++.h>
#include <mbed.h>
#include <USBSerial.h>

struct GCSCoordinates {
    int64_t latitude;
    int64_t longitude;
};

class GPSManager{
    public:
        GPSManager(mbed::UnbufferedSerial& serial_connection, TinyGPSPlus& gps_parser); 
        void readData();
        TinyGPSPlus& getGPS();
        void callback();
        GCSCoordinates getLocation();
        void testPrint(USBSerial& serial);

    private:
        mbed::UnbufferedSerial& serial_connection;
        TinyGPSPlus& gps_parser;
        uint8_t c;   
        void updateParser(char data);
        int64_t rawToInt(RawDegrees raw);
};
