#include "Pump.h"

Pump::Pump(uint8_t pin_pump) : pin_pump_(pin_pump), stateTimeStamp_(0) {
    pinMode(pin_pump_, OUTPUT);
    digitalWrite(pin_pump_, LOW);
    currentState_ = State::OFF;
}

Pump::State Pump::getState() {
    return currentState_;
}

void Pump::setState(State state) {
    if (state == State::ON) {
        digitalWrite(pin_pump_, HIGH);
    } else {
        digitalWrite(pin_pump_, LOW);
    }
    currentState_ = state;
    stateTimeStamp_ = millis();
}

void Pump::on() {
    setState(State::ON);
}

void Pump::off() {
    setState(State::OFF);
}
