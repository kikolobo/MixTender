#include "Arduino.h"
#include <ESP32Servo.h>
#include <memory>

#pragma once

class Valve
{
public:
    enum class Position {
        CLOSED,
        OPEN,
    };

    Valve(uint8_t pin_servo, uint32_t closedPosition = 54, uint32_t openPosition = 155, uint32_t frequency = 300); //140
    Position getPosition();    
    void setPosition(Position position);
    void writeValue(uint32_t value);
    void trimOpen(uint32_t value);
    void trimClosed(uint32_t value);
    uint32_t getOpenPosition();
    uint32_t getClosedPosition();
    uint32_t getOpenPositionTrim();
    uint32_t getClosedPositionTrim();
    void resetTrimPositions();
    
private:
    Servo servo_;
    Valve::Position position_;
    uint8_t pin_servo_;
    uint8_t timerChannel_;
    uint32_t closedPosition_;
    uint32_t openPosition_;
    uint32_t openPositionTrim_;
    uint32_t closedPositionTrim_;
    uint32_t currentPosition_;
    uint32_t positionTimeStamp_;
};
