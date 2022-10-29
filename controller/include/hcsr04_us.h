#pragma once

#include <gpio_interface.h>
// #include <chrono>

class HCRS04Ultrasonic {
    public:
        HCRS04Ultrasonic(GPIOOutputInterface& trigger);
        void triggerPulse();
        uint32_t getDistanceMm(uint32_t microsec_count);
        bool shouldTrigger();
        bool readReady();

    private:
        GPIOOutputInterface& trigger;
        void echoRise();
        void echoFall();
        // std::chrono::microseconds t;
        uint64_t c_nm_us_half = 171500;
        bool updated = false;
        bool triggered = false;
        bool isUpdated();
        bool isTriggered();
};