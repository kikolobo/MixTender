#include "Arduino.h"

#pragma once

class Pump
{
public:
    enum class State {
        OFF,
        ON,
    };

    Pump(uint8_t pin_pump); //140
    State getState(); 
    void setState(State state);
    void on();
    void off();   
    
private:    
    Pump::State position_;
    uint8_t pin_pump_;    
    State currentState_;
    uint32_t stateTimeStamp_;
};
