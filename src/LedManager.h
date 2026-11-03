

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
bool fadedComplete();
bool isFading();
CRGB getCurrentColor() { return currentColor_; }
FX getCurrentFX() { return currentFX_; }
   
        
private:    
    uint8_t calculateBrightness_(CRGB color);
    void performFade_();
    void show_();   // request a frame (rate-limited)
    void flush_();  // push a pending frame if the interval has passed
    static const uint32_t MAX_FPS = 30;
    static const uint32_t FRAME_INTERVAL_MS = 1000 / MAX_FPS;
    uint32_t lastShowMs_ = 0;
    bool dirty_ = false;
    int numLeds_;
    uint8_t dataPin_;  
    int prevTimeStamp_; 
    CRGB leds_[76];
    CRGB backgroundColor_;
    FX currentFX_ = FX::NONE;
    uint32_t FXdurationMS_ = 0;
    uint32_t FXPeriodMS_;
    uint32_t startTimestamp_ = 0;
    bool isUniform_ = true; // All LEDs show currentColor_ (false after a range/tray pattern).
    CRGB startColor_;
    CRGB targetColor_;
    CRGB currentColor_; 
    CRGB colorWheel_[3] = {CRGB::Red, CRGB::Green, CRGB::Blue}; 
    uint8_t currentBrightness_;   
    uint8_t showStepIndex_ = 0;   
};
