#include <inttypes.h>

class SPIBusMaster {
    public:
        virtual void lock();
        virtual void unlock();
        virtual uint8_t write(uint8_t data);
        virtual void write(uint8_t tx_buffer[], uint32_t tx_length, uint8_t rx_buffer[], uint32_t rx_length);
};