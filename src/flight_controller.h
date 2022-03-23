#pragma once

#include <ms5611.h>

struct GpsCoord {
    int32_t latitude;
    int32_t longitude;
};

struct Attitude {
    int32_t yaw;
    int32_t pitch;
    int32_t roll;
};

class FlightController {
    public:
        FlightController(GpsCoord home_position);
        void updateMotorOutputs();
        void updateAttitude(Attitude attitude);
        void updateTempPressure(TEMP_AND_PRESSURE tempAndPressure);
        
        void updateSettings(bool auto_pilot, bool altitude_hold);
    private:
        const GpsCoord home_position;
        GpsCoord gps_set_point;
        Attitude attitude;
        int16_t altitude_set_point;
        bool auto_pilot;
        bool altitude_hold;
};