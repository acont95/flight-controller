#include "hcsr04_us.hpp"


HCRS04Ultrasonic::HCRS04Ultrasonic(GPIOOutputInterface& trigger, TimerInterface& timer, SleepInterface& sleeper) : trigger(trigger), timer(timer), sleeper(sleeper){

}

void HCRS04Ultrasonic::triggerPulse() {
    updated = false;
    triggered = true;
    trigger.setHigh();
    sleeper.sleepUs(20);
    trigger.setLow();
    // updated = false;
    // triggered = true;
}

uint32_t HCRS04Ultrasonic::getDistanceMm() {
    updated = false;
    return (t * c_nm_us_half) / 1000000;
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

TimerInterface& HCRS04Ultrasonic::getTimer() {
    return timer;
}

void HCRS04Ultrasonic::echoRise() {
    timer.start();
}

void HCRS04Ultrasonic::echoFall() {
    updated = true;
    triggered = false;
    timer.stop();
    t = timer.elapsedUs();
    // updated = true;
    // triggered = false;
}