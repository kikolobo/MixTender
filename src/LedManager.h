

#include <Arduino.h>
#include <FastLED.h>
#include <memory>

#pragma once

class LedManager
{    
public:    

enum class FX {
    NONE,
    FADEIN
};

LedManager(uint8_t dataPin);
void heartbeat();

void setAllLeds(CRGB color, bool shouldRender = true);
void setLed(int index, CRGB color);
void clearAllLeds();
void clearLed(int index);
void fadeTo(CRGB startColor, CRGB targetColor, int durationMS);
void setRange(uint8_t start, uint8_t end, CRGB color);
void trackTray(uint32_t position, CRGB color, CRGB backgroundColor);
void fadeShow();
CRGB getCurrentColor() { return currentColor_; }
   
        
private:    
    uint8_t calculateBrightness_(CRGB color);
    void performFade_();


    int numLeds_;
    uint8_t dataPin_;  
    int prevTimeStamp_; 
    CRGB leds_[76];
    CRGB backgroundColor_;
    FX currentFX_;
    uint32_t FXdurationMS_;
    uint32_t FXPeriodMS_;
    CRGB startColor_;
    CRGB targetColor_;
    CRGB currentColor_; 
    CRGB colorWheel_[3] = {CRGB::Red, CRGB::Green, CRGB::Blue}; 
    uint8_t currentBrightness_;   
    uint8_t showStepIndex_ = 0;   
};
