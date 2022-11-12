#include <pico_timer_impl.hpp>

PicoTimer::PicoTimer() {}

void PicoTimer::start() {
    start_time = get_absolute_time();
}

void PicoTimer::stop() {
    stop_time = get_absolute_time();
}

uint64_t PicoTimer::elapsedUs() {
    return absolute_time_diff_us(start_time, stop_time);
}