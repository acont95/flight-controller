#pragma once

#include <inttypes.h>

class UARTInterface {
    public:
        virtual void put(char c);
        virtual char get();
};