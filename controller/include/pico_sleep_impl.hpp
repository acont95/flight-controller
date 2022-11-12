#pragma once

#include <stdio.h>
#include <pico/stdlib.h>
#include <pico/time.h>
#include <sleep_interface.hpp>


class PicoSleepImpl : public SleepInterface {
    public:
        PicoSleepImpl();
        void sleepMs(uint64_t ms);
        void sleepUs(uint64_t us);
};