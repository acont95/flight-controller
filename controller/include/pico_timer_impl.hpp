#pragma once

#include <timer_interface.hpp>
#include <pico/time.h>

class PicoTimer: public TimerInterface {
    public:
        PicoTimer();
        void start();
        void stop();
        uint64_t elapsedUs();
    private:
        absolute_time_t start_time;
        absolute_time_t stop_time;
};
