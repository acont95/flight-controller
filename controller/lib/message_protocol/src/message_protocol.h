#pragma once

#include <cstdint>

class MessageProtocol {
    public:
        bool decodeData(uint8_t* buffer, uint8_t size, uint8_t* bytes);
        bool encodeData(uint8_t* bytes, uint8_t size, uint8_t* buffer);
};