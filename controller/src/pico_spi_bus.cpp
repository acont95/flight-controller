#include <pico_spi_bus.hpp>

PicoSPIBus::PicoSPIBus(spi_inst_t& spi_instance) : spi_instance(spi_instance){}

uint8_t PicoSPIBus::write(uint8_t data) {
    uint8_t val;
    spi_read_blocking(&spi_instance, 0, &val, 1);
    return val;
}

void PicoSPIBus::write(uint8_t tx_buffer[], uint32_t tx_length, uint8_t rx_buffer[], uint32_t rx_length) {

}

void PicoSPIBus::unlock() {
    
}

void PicoSPIBus::lock() {

}
