#include <message_protocol.h>
#include <mbed.h>
#include <platform/CircularBuffer.h>
#include <flight_controller.h>
#include <imu_manager.h>
#include <gps_manager.h>

#define BUFFER_SIZE 64
#define CHECK_SUM_LOCATION 16

struct FlightCommand {
    Attitude attitude;
    uint8_t thrust;
};

struct Telemetry {
    Attitude attitude;
    GCSCoordinates location;
    int32_t altitude;
    uint16_t height_above_ground;
};

enum class MessageType {
    FLIGHT_COMMAND,
    RETURN_TO_HOME,
    TAKE_OFF,
    LAND
};

struct Message {
    bool check_sum;
    MessageType message_type;
    FlightCommand flight_command;
};

class MessageManager {
    public:
        MessageManager(MessageProtocol& message_protocol);
        void encode(uint8_t data);
        bool messageReady();
        void sendTelemetry(Telemetry telemetry);
        Message getMessage();

    private:
        mbed::CircularBuffer<uint8_t, BUFFER_SIZE> circ_buf;    
        uint8_t buf[BUFFER_SIZE];
        uint8_t decoded[BUFFER_SIZE];
        void decodeMessage();
        bool validCheckSum();
        MessageProtocol& message_protocol;
        uint8_t start_byte;
        bool message_ready;
        uint16_t checkSum(uint8_t buf[], uint16_t size);
        uint16_t readCheckSum();
        uint16_t readLength();
        MessageType readMessageType();
};
