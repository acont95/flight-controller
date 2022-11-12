#pragma once

#include <chrono>
#include <gpio_output_interface.hpp>
#include <timer_interface.hpp>
#include <sleep_interface.hpp>

class HCRS04Ultrasonic {
    public:
        HCRS04Ultrasonic(GPIOOutputInterface& trigger, TimerInterface& timer, SleepInterface& sleeper);
        void triggerPulse();
        uint32_t getDistanceMm();
        bool shouldTrigger();
        bool readReady();
        TimerInterface& getTimer();
        void echoRise();
        void echoFall();

    private:
        GPIOOutputInterface& trigger;
        TimerInterface& timer;
        SleepInterface& sleeper;
        uint64_t t;
        uint64_t c_nm_us_half = 171500;
        bool updated = false;
        bool triggered = false;
        bool isUpdated();
        bool isTriggered();
};