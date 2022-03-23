#include <TinyGPS++.h>
#include <mbed.h>

class GPSManager{
    public:
        GPSManager(mbed::UnbufferedSerial& serial_connection, TinyGPSPlus& gps_parser); 
        void readData();
        TinyGPSPlus& getGPS();

    private:
        mbed::UnbufferedSerial& serial_connection;
        TinyGPSPlus& gps_parser;
        uint8_t c;   
        void updateParser(char data);
};
