typedef struct GpsCoord {
    int32_t latitude;
    int32_t longitude;
} GpsCoord;

typedef struct Attitude {
    int16_t yaw;
    int16_t pitch;
    int16_t roll;
} Attitude;

class FlightController {
    public:
        FlightController(GpsCoord home_position);
        void updateMotorOutputs();
        void updateYawPitchRoll(int16_t yaw, int16_t pitch, int16_t roll);
        void updateSettings(bool auto_pilot, bool altitude_hold);
    private:
        const GpsCoord home_position;
        GpsCoord gps_set_point;
        Attitude attitude;
        int16_t altitude_set_point;
        bool auto_pilot;
        bool altitude_hold;
};