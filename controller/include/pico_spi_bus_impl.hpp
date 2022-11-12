#pragma once

#include <stdio.h>
#include <pico/stdlib.h>
#include <hardware/spi.h>
#include <inttypes.h>
#include <spi_bus.hpp>


class PicoSPIBus: public SPIBusMaster {
    public:
        PicoSPIBus(spi_inst_t* spi_instance, uint baud_rate);
        void lock();
        void unlock();
        uint8_t write(uint8_t data);
        void write(uint8_t tx_buffer[], uint32_t tx_length, uint8_t rx_buffer[], uint32_t rx_length);
        void init();
    private:
        spi_inst_t* spi_instance;
        uint baud_rate;
};