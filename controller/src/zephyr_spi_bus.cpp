#include <zephyr_spi_bus.hpp>

ZephyrSPIBus::ZephyrSPIBus(spi_dt_spec* spi_spec) : spi_spec(spi_spec){
}

uint8_t ZephyrSPIBus::write(uint8_t data) {
    spi_buf tx_bufs[] = {
        {
                .buf = &data,
                .len = 1
        },
    };

    spi_buf_set tx = {
            .buffers =  tx_bufs,
            .count = 1
    };

    uint8_t rx_buf_init = 0;

    spi_buf rx_bufs[] = {
        {
                .buf = &rx_buf_init,
                .len = 1
        },
    };

    spi_buf_set rx = {
            .buffers =  rx_bufs,
            .count = 1
    };

    spi_transceive_dt(spi_spec, &tx, &rx);
    return ((intptr_t*)(rx.buffers[0].buf))[0];
}

void ZephyrSPIBus::write(uint8_t tx_buffer[], uint32_t tx_length, uint8_t rx_buffer[], uint32_t rx_length) {
    spi_buf tx_bufs[] = {
        {
                .buf = tx_buffer,
                .len = tx_length
        },
    };

    spi_buf_set tx = {
            .buffers =  tx_bufs,
            .count = 1
    };

    spi_buf rx_bufs[] = {
        {
                .buf = rx_buffer,
                .len = rx_length
        },
    };

    spi_buf_set rx = {
            .buffers =  rx_bufs,
            .count = 1
    };

    spi_transceive_dt(spi_spec, &tx, &rx);
}

void ZephyrSPIBus::unlock() {
    spi_release_dt(spi_spec);
}

void ZephyrSPIBus::lock() {

}
