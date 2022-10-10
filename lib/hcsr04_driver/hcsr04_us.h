#pragma once

#include <mbed.h>
#include <USBSerial.h>
#include <platform/mbed_wait_api.h>


class HCRS04Ultrasonic {
    public:
        HCRS04Ultrasonic(const PinName& echo_pin, mbed::DigitalOut& trigger);
        void triggerPulse();
        uint32_t getDistanceMm();
        bool shouldTrigger();
        bool readReady();
        void testPrint(USBSerial& serial);

    private:
        mbed::InterruptIn echo_int;
        mbed::Timer timer;
        mbed::DigitalOut& trigger;
        void echoRise();
        void echoFall();
        std::chrono::microseconds t;
        uint64_t c_nm_us_half = 171500;
        bool updated = false;
        bool triggered = false;
        bool isUpdated();
        bool isTriggered();
};