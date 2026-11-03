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

    Valve(uint8_t pin_servo, uint32_t closedPosition = 50, uint32_t openPosition = 147, uint32_t frequency = 300); //140
    Position getPosition();    
    void setPosition(Position position);
    void writeValue(uint32_t value);
    // Trims are signed offsets (degrees) added to the base open/closed angles.
    // trimOpen/trimClosed also move the valve there; setTrims does not move it.
    void trimOpen(int32_t value);
    void trimClosed(int32_t value);
    void setTrims(int32_t openTrim, int32_t closedTrim);
    uint32_t getOpenPosition();
    uint32_t getClosedPosition();
    int32_t getOpenPositionTrim();
    int32_t getClosedPositionTrim();
    uint32_t getOpenAngle();   // base + trim, clamped to 0..180
    uint32_t getClosedAngle();
    void resetTrimPositions();
    
private:
    Servo servo_;
    Valve::Position position_;
    uint8_t pin_servo_;
    uint8_t timerChannel_;
    uint32_t closedPosition_;
    uint32_t openPosition_;
    int32_t openPositionTrim_;
    int32_t closedPositionTrim_;
    int32_t clampTrim_(uint32_t base, int32_t trim);
    uint32_t currentPosition_;
    uint32_t positionTimeStamp_;
};
