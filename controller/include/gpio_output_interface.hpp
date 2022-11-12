#pragma once

#include <inttypes.h>

class GPIOOutputInterface {
    public:
        virtual void setHigh();
        virtual void setLow();
        virtual void toggle();
};