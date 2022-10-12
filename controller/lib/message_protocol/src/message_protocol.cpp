#include "message_protocol.h"


bool MessageProtocol::decodeData(uint8_t buffer[], uint8_t size, uint8_t bytes[]) {

    uint16_t decode_pointer = 0;
    uint16_t count = 0;
    uint8_t code = 0xff;
    
    for (int i = 0; i<size; i++) {
        if (count) {
            bytes[decode_pointer] = buffer[i];
            decode_pointer++;
        } else {
            if ((code != 0xff) || (buffer[i] != 0)) {
                bytes[decode_pointer] = 0;
                decode_pointer++;
            }
            count = code = buffer[i];
            if (!code){
                break;
            }
        }
        count--;
    }
    return true;
}

bool MessageProtocol::encodeData(uint8_t bytes[], uint8_t size, uint8_t buffer[]) {
    uint16_t cur = 0;
    uint16_t count = 1;
    uint16_t buffer_pointer = 1;

    for (uint16_t i=0; i<size; i++) {
        if (bytes[i]) {
            buffer[buffer_pointer] = bytes[i];
            count++;
            buffer_pointer++;
        }
        if ((!bytes[i]) || (count == 0xff)) {
            buffer[cur] = count;
            count = 1;
            cur = buffer_pointer;
            if ((!bytes[i]) || (i < size-1)) {
                buffer_pointer++;
            }
        }
    }

    return true;
}