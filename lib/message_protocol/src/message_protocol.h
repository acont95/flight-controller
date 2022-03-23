#pragma once

#include <cstdint>
#define MAX_MESSAGE_SIZE 254

class MessageProtocol {
    public:
        // MessageProtocol(uint8_t max_message_size);
        bool decodeData(uint8_t* buffer, uint8_t size, uint8_t* bytes);
        bool encodeData(uint8_t* bytes, uint8_t size, uint8_t* buffer);
        uint8_t* getMessage();

    private:
        static const uint8_t start_byte = 0x3C;
        static const uint8_t end_byte = 0x3E;
        static const uint8_t escape_byte = 0x5C;
        uint8_t buffer[MAX_MESSAGE_SIZE] = {0};
        bool frame_started = false;
        uint16_t count = 0;
};