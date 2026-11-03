#include "Valve.h"

Valve::Valve(uint8_t pin_servo, uint32_t closedPosition, uint32_t openPosition, uint32_t frequency) {
    pin_servo_ = pin_servo;    
    closedPosition_ = closedPosition;
    openPosition_ = openPosition;
    closedPositionTrim_ = 0;
    openPositionTrim_ = 0;
    position_ =  Position::CLOSED;
    // ESP32PWM::allocateTimer(0);
    
    servo_.attach(pin_servo_, 500, 2500);
    servo_.setPeriodHertz(frequency);
    setPosition(Position::CLOSED);
}

Valve::Position Valve::getPosition() {
    return position_;
}

uint32_t Valve::getOpenPosition() {
    return openPosition_;
}

uint32_t Valve::getClosedPosition() {
    return closedPosition_;
}

void Valve::writeValue(uint32_t value) {
    servo_.write(value);
}

// Keep base + trim inside the servo's 0..180 range.
int32_t Valve::clampTrim_(uint32_t base, int32_t trim) {
    return constrain(trim, -(int32_t)base, 180 - (int32_t)base);
}

void Valve::trimOpen(int32_t value) {
    openPositionTrim_ = clampTrim_(openPosition_, value);
    setPosition(Position::OPEN);
}

void Valve::trimClosed(int32_t value) {
   closedPositionTrim_ = clampTrim_(closedPosition_, value);
   setPosition(Position::CLOSED);
}

void Valve::setTrims(int32_t openTrim, int32_t closedTrim) {
    openPositionTrim_ = clampTrim_(openPosition_, openTrim);
    closedPositionTrim_ = clampTrim_(closedPosition_, closedTrim);
}

int32_t Valve::getOpenPositionTrim() {
    return openPositionTrim_;
}

int32_t Valve::getClosedPositionTrim() {
    return closedPositionTrim_;
}

uint32_t Valve::getOpenAngle() {
    return openPosition_ + openPositionTrim_;
}

uint32_t Valve::getClosedAngle() {
    return closedPosition_ + closedPositionTrim_;
}

void Valve::resetTrimPositions() {
    openPositionTrim_ = 0;
    closedPositionTrim_ = 0;

}


void Valve::setPosition(Position position) {
    // position_ = position == Position::CLOSED ? 0 : 1;
    position_ = position;
    currentPosition_ = (position_ == Position::CLOSED) ? getClosedAngle() : getOpenAngle();
    servo_.write(currentPosition_);
    Serial.print("[Valve][setPosition] Setting valve position to: ");
    Serial.print(position_ == Position::CLOSED ? "CLOSED" : "OPEN");
    Serial.println(" at: " + String(currentPosition_));
    
    positionTimeStamp_ = millis();
}