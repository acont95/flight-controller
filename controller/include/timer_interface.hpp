#pragma once

#include <inttypes.h>

class TimerInterface {
    public:
        virtual void start();
        virtual void stop();
        virtual uint64_t elapsedUs();
};