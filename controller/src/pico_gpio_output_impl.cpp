#include <pico_gpio_output_impl.hpp>

PicoGPIOOutputImpl::PicoGPIOOutputImpl(uint pin) : pin(pin){}

void PicoGPIOOutputImpl::setHigh() {
    gpio_put(pin, true);
}

void PicoGPIOOutputImpl::setLow() {
    gpio_put(pin, false);
}

void PicoGPIOOutputImpl::toggle() {
    if(gpio_get_out_level(pin)) {
        setLow();
    } else {
        setHigh();
    }
}

void PicoGPIOOutputImpl::init() {
    gpio_init(pin);
    gpio_set_dir(pin, true);
}