#include <pico_sleep_impl.hpp>

PicoSleepImpl::PicoSleepImpl() {}

void PicoSleepImpl::sleepMs(uint64_t ms) {
    sleep_ms(ms);
}

void PicoSleepImpl::sleepUs(uint64_t us) {
    sleep_us(us);
}