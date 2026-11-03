#include "LedManager.h"

LedManager::LedManager(uint8_t dataPin) {
    numLeds_ = 75;
    dataPin_ = dataPin;    
    FastLED.addLeds<NEOPIXEL, SDA>(leds_, 75);  // GRB ordering is assumed    
    FastLED.clear();
    FastLED.show();        
    FastLED.setBrightness(100);
    currentFX_ = FX::NONE;
}


void LedManager::heartbeat() {
    if (currentFX_ == FX::FADEIN) {
        performFade_();
    }
}

void LedManager::setAllLeds(CRGB color, bool shouldRender) {
    currentColor_ = color;
    isUniform_ = true;
    backgroundColor_ = color;
    currentBrightness_ = calculateBrightness_(color);    
    bool changesExist = false;
    for (int i = 0; i < numLeds_; i++) {
        if (leds_[i] != color) {            
            leds_[i] = color; // Correcting previous mistake: Direct assignment is used, not ->setRGB        
            changesExist = true;
        }
    }

    if (shouldRender == true) {
        if (changesExist == true) {
         FastLED.show();  
        }
    }    
}

void LedManager::setLed(int index, CRGB color) {
    isUniform_ = false;
    leds_[index] = color; // Again, correcting: Direct assignment is correct
    FastLED.show();
}

void LedManager::clearAllLeds() {
    currentColor_ = CRGB::Black; // Using CRGB::Black for clarity
    isUniform_ = true;

    for (int i = 0; i < numLeds_; i++) {
        leds_[i] = CRGB::Black; // Using CRGB::Black for clarity
    }
    FastLED.show();
}

void LedManager::clearLed(int index) {
    isUniform_ = false;
    leds_[index] = CRGB::Black; // Using CRGB::Black for clarity
    FastLED.show();
}
 
 
 void LedManager::fadeTo(CRGB startColor, CRGB targetColor, int durationMS) {
        // Already fading there, or already showing exactly that color.
        // (Not just "same target as last time": a tray-tracking pattern may
        // have been drawn since, and then the fade must run again.)
        if (currentFX_ == FX::FADEIN && targetColor == targetColor_) {
            return;
        }
        if (currentFX_ == FX::NONE && isUniform_ && targetColor == currentColor_) {
            return;
        }


        targetColor_ = targetColor;
        currentFX_ = FX::FADEIN;
        FXdurationMS_ = durationMS;
        startColor_ = startColor;
        setAllLeds(startColor);
        startTimestamp_ = millis(); // Capture the start time of the fade
    }

    void LedManager::performFade_() {
        uint32_t elapsed = millis() - startTimestamp_;
        // Clamp so the last frame always lands on the target. Before, the
        // fade only ended if a frame hit the exact final millisecond, so it
        // usually never finished and stopped just short of the target color.
        bool done = (FXdurationMS_ == 0) || (elapsed >= FXdurationMS_);

        CRGB newColor = targetColor_;
        if (!done) {
            uint8_t frac = (uint8_t)((elapsed * 255UL) / FXdurationMS_);
            newColor = CRGB(
                lerp8by8(startColor_.r, targetColor_.r, frac),
                lerp8by8(startColor_.g, targetColor_.g, frac),
                lerp8by8(startColor_.b, targetColor_.b, frac)
            );
        }

        for (int j = 0; j < numLeds_; j++) {
            leds_[j] = newColor;
        }
        currentColor_ = newColor;
        isUniform_ = true;
        FastLED.show();

        if (done) {
            currentFX_ = FX::NONE;
        }
    }
// void LedManager::fadeTo(CRGB startColor, CRGB targetColor, int durationMS) {    
//     if (targetColor == targetColor_ || targetColor == currentColor_) {        
//         return;
//     }

//     targetColor_ = targetColor;    
//     currentFX_ = FX::FADEIN;
//     FXdurationMS_ = durationMS;
//     FXPeriodMS_ = durationMS / 500;
//     currentBrightness_ = 0;
//     startColor_ = startColor;    
//     setAllLeds(startColor);
// }


// void LedManager::performFade_() {    
//     if (millis() - prevTimeStamp_ > FXPeriodMS_) {
//         // Calculate the progress ratio (0.0 to 1.0)
//         float progress = (float)currentBrightness_ / 255.0;
        
//         // Interpolate between startColor_ and targetColor_ based on progress
//         CRGB newColor = CRGB(
//             startColor_.r + (targetColor_.r - startColor_.r) * progress,
//             startColor_.g + (targetColor_.g - startColor_.g) * progress,
//             startColor_.b + (targetColor_.b - startColor_.b) * progress
//         );
        
//         // Apply the new color to all LEDs
//         for (int j = 0; j < numLeds_; j++) {
//             leds_[j] = newColor;
//         }
//         currentColor_ = newColor;
//         FastLED.show(); // Update the LEDs
            
//         prevTimeStamp_ = millis();
//         currentBrightness_ += 1;
        
//         // End the effect once the progress completes
//         if (currentBrightness_ >= 255) {
//             currentFX_ = FX::NONE;
//             currentColor_ = targetColor_;
//         }   
//     }        
// }


uint8_t LedManager::calculateBrightness_(CRGB color) {
    return max(color.r, max(color.g, color.b));
}

void LedManager::setRange(uint8_t start, uint8_t end, CRGB color) {
    currentFX_ = FX::NONE; // A drawn pattern replaces any running fade (they used to fight each frame).
    setAllLeds(backgroundColor_, false);
    isUniform_ = false;
    for (int i = start; i < end && i < numLeds_; i++) {
        leds_[i] = color;
    }
    FastLED.show();
}

void LedManager::trackTray(uint32_t position, CRGB color, CRGB backgroundColor) {
    backgroundColor_ = backgroundColor;
    int ledIndex = map(position, 2340, 0, 0, numLeds_); 
    if (ledIndex<=14) {ledIndex = 14;}
    setRange(ledIndex-14, ledIndex, color);    
}

bool LedManager::fadedComplete() {
    return currentFX_ == FX::NONE && isUniform_ && currentColor_ == targetColor_;
}

bool LedManager::isFading() {
    return currentFX_ == FX::FADEIN;
}

// 