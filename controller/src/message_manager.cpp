#include <message_manager.h>


MessageManager::MessageManager(MessageProtocol& message_protocol): message_protocol(message_protocol) {

}

void MessageManager::encode(uint8_t data) {
    circ_buf.push(data);
    if (data == 0) {
        message_ready = true;
    }
}

bool MessageManager::messageReady() {
    return message_ready;
}

void MessageManager::decodeMessage() {
    uint32_t size = circ_buf.size();
    circ_buf.pop(buf, size);
    message_protocol.decodeData(buf, size, decoded);
}

bool MessageManager::validCheckSum() {
    return checkSum(decoded, sizeof(decoded)) == readCheckSum();
}

uint16_t MessageManager::readCheckSum() {
    return (decoded[2] << 8) | decoded[3];
}

uint16_t MessageManager::readLength() {
    return (decoded[0] << 8) | decoded[1];
}

MessageType MessageManager::readMessageType() {
    uint8_t message_type_num = decoded[4];
    switch (message_type_num) {
        case 0x00:
            return MessageType::FLIGHT_COMMAND;
        case 0x01:
            return MessageType::RETURN_TO_HOME;
        case 0x02:
            return MessageType::TAKE_OFF;
        case 0x03:
            return MessageType::LAND;
    }
}

uint16_t MessageManager::checkSum(uint8_t buf[], uint16_t size) {
    uint16_t lrc = 0;
    for (int i=0; i < size; i++) {
        lrc = (lrc + buf[i]) & 0xff;
    }
    return (((lrc ^ 0xff) + 1) & 0xff);
}

Message MessageManager::getMessage() {
    Message message;
    decodeMessage();
    message.message_type = readMessageType();
    message.check_sum = validCheckSum();

    // switch(message.message_type) {
    //     case MessageType::FLIGHT_COMMAND:
    //         message.flight_command.attitude.roll = (decoded[5] << 8) | decoded[6];
    //         message.flight_command.attitude.pitch = (decoded[7] << 8) | decoded[8];
    //         message.flight_command.attitude.yaw = (decoded[8] << 8) | decoded[9];
    //         message.flight_command.thrust = decoded[10];
    //         break;
    // }
    return message;
    
}