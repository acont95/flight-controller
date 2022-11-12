#pragma once

#include <stdio.h>
#include <pico/stdlib.h>
#include <hardware/gpio.h>
#include <gpio_output_interface.hpp>

class PicoGPIOOutputImpl: public GPIOOutputInterface {
    public:
        PicoGPIOOutputImpl(uint pin);
        void setHigh();
        void setLow();
        void toggle();
        void init();
    private:
        uint pin;
};