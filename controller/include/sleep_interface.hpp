#pragma once

#include <inttypes.h>

class SleepInterface {
    public:
        virtual void sleepMs(uint64_t ms);
        virtual void sleepUs(uint64_t us);
};