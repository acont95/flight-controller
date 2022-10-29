#include "hcsr04_us.h"


HCRS04Ultrasonic::HCRS04Ultrasonic(GPIOOutputInterface& trigger):trigger(trigger) {
    // echo_int.rise(mbed::callback(this, &HCRS04Ultrasonic::echoRise));
    // echo_int.fall(mbed::callback(this, &HCRS04Ultrasonic::echoFall));
}

void HCRS04Ultrasonic::triggerPulse() {
    trigger.setHigh();
    // wait_us(10);
    trigger.setLow();
    updated = false;
    triggered = true;
}

void HCRS04Ultrasonic::echoRise() {
    // timer.start();
}

void HCRS04Ultrasonic::echoFall() {
    // timer.stop();
    // t = timer.elapsed_time();
    // timer.reset();
    updated = true;
    triggered = false;
}

uint32_t HCRS04Ultrasonic::getDistanceMm(uint32_t microsec_count) {
    updated = false;
    return (microsec_count * c_nm_us_half) / 1000000;
}

bool HCRS04Ultrasonic::isUpdated() {
    return updated;
}

bool HCRS04Ultrasonic::isTriggered() {
    return triggered;
}

bool HCRS04Ultrasonic::shouldTrigger() {
    return !isTriggered() && !isUpdated();
}

bool HCRS04Ultrasonic::readReady() {
    return !isTriggered() && isUpdated();
}

// void HCRS04Ultrasonic::testPrint(USBSerial& serial) {
//     if (shouldTrigger()) {
//         triggerPulse();
//     } 
//     if (readReady()) {
//         uint32_t d_mm = getDistanceMm();
//         serial.printf("Ultrasonic Sensor (mm): %i \n", (int)d_mm);
//     }
// }