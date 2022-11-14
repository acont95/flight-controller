#include <pico_spi_bus_impl.hpp>

PicoSPIBus::PicoSPIBus(spi_inst_t* spi_instance, SemaphoreHandle_t* mutex, uint baud_rate) : spi_instance(spi_instance), mutex(mutex), baud_rate(baud_rate) {}

uint8_t PicoSPIBus::write(uint8_t data) {
    uint8_t val;
    spi_write_read_blocking(spi_instance, &data, &val, 1);
    return val;
}

void PicoSPIBus::write(uint8_t tx_buffer[], uint32_t tx_length, uint8_t rx_buffer[], uint32_t rx_length) {
    spi_write_read_blocking(spi_instance, tx_buffer, rx_buffer, tx_length);
}

void PicoSPIBus::unlock() {
    xSemaphoreGive(*mutex);
}

void PicoSPIBus::lock() {
    xSemaphoreTake(*mutex, portMAX_DELAY);
}

void PicoSPIBus::init() {
    spi_init(spi_instance, baud_rate);
}
