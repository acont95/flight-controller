// #define DT_DRV_COMPAT raspberrypi_pico_spi

#include <spi_bus.h>
#include <zephyr/drivers/spi.h>

class ZephyrSPIBus: public SPIBusMaster {
    public:
        ZephyrSPIBus(spi_dt_spec* spi_spec);
        void lock();
        void unlock();
        uint8_t write(uint8_t data);
        void write(uint8_t tx_buffer[], uint32_t tx_length, uint8_t rx_buffer[], uint32_t rx_length);
    private:
        spi_dt_spec* spi_spec;
};