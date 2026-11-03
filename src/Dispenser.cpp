#include "Dispenser.h"
#include <Preferences.h>

// Valve trims are stored in NVS (flash) under this namespace, keys "v<idx>o" / "v<idx>c".
static const char* TRIM_NVS_NAMESPACE = "valvetrim";



Dispenser::Dispenser(uint8_t dat_pin, uint8_t sck_pin, float kFactor,  float emptyWeight ) {
    scale_ = std::make_unique<HX711>();
    scale_->begin(dat_pin, sck_pin);
    scale_->set_scale(kFactor);
    valves_.clear();
    emptyWeight_ = emptyWeight;      
    valveIndex_ = 0;
    pumpIndex_ = 0;
    pourDeviceIndex_ = 0;
    dispenseType_ = DispenseType::VALVE;
    state_ = DispenserState::READY;
    targetWeight_ = 0.0;
    latestWeight_ = 0.0;
    filteredValue_ = 0.0;
    awaitingClosureTimeStampMS_ = 0;
    awaitingStabilityTimeStampMS_ = 0;
    Serial.println("[Dispenser][Constructor] Dispenser created: " + String(scale_->get_units(1)));
};


void Dispenser::heartbeat() {
    
    if (scale_->is_ready()) {
        float alpha = 0.4; // Smoothing factor. Adjust as needed.
        float rawValue = scale_->get_units(1); // Get the current raw value        
        filteredValue_ = alpha * rawValue + (1 - alpha) * filteredValue_; // Apply the low-pass filter

        float resolution = 0.14; // Target resolution, e.g., 0.1 kg
        float finalValue = round(filteredValue_ / resolution) * resolution; // Reduce resolution


        latestWeight_ = finalValue; //fabs(scale_->get_units(5));
        // Serial.println("[Dispenser][heartbeat] Latest weight: " A+ String(latestWeight_) + "g");
    }

    if (state_ == DispenserState::READY) {
        return; //Skip all the noise if we're not dispensing.
    }

    if (state_ == DispenserState::FINISHED) {
        return; //Skip all the noise if we're done dispensing.
    }

    if (state_ == DispenserState::AWAITING_CLOSURE) {
        if (millis() - awaitingClosureTimeStampMS_ > 1000) {
            Serial.println("[Dispenser][AWAITING_CLOSURE] Awaiting closure complete.");
            resetDispensing_();
        }
    }

    if (state_ == DispenserState::AWAITING_STABILITY) {
        if (millis() - awaitingStabilityTimeStampMS_ > 500) {
            tare(); // Tare only once the tray has settled after moving.
            state_ = DispenserState::STABLE;
        }
    }

    if (state_ == DispenserState::STABLE) {             
            Serial.println("[Dispenser][STABLE] Station Begin weight: " + String(getLatestWeight()) + "g");            
            state_ = DispenserState::DISPENSING;
            if (dispenseType_ == DispenseType::PUMP) {
                pumps_[pourDeviceIndex_]->on();
            } else {
                valves_[pourDeviceIndex_]->setPosition(Valve::Position::OPEN);                            
            }
    }

    if (state_ == DispenserState::DISPENSING) {
        if (latestWeight_ >= targetWeight_) {
            Serial.println("[Dispenser][DISPENSING] Dispensing complete.");
            finishDispensing_();
        }
    }
};

void Dispenser::tare() {
    Serial.println("[Dispenser][tare] Scale Tared.");
    scale_->tare();
    // Drop the pre-tare history from the low-pass filter, otherwise the old
    // reading decays into the new step and can end small pours early.
    filteredValue_ = 0.0;
    latestWeight_ = 0.0;
};


void Dispenser::selectValveForTrim(uint32_t valveId, Valve::Position position) {
    if (valveId < 1 || valveId > valves_.size()) {
        Serial.println("[Dispenser][selectValveForTrim] Invalid valve: " + String(valveId) + " (1.." + String(valves_.size()) + ")");
        return;
    }

    trimValveIndex_ = valveId - 1;
    // Move to the position with its current trim kept (it used to reset to 0).
    valves_[trimValveIndex_]->setPosition(position);
    printValveTrim_(trimValveIndex_);
};

void Dispenser::printValveTrim_(uint8_t index) {
    const auto& v = valves_[index];
    bool saved = isValveTrimSaved_(index);
    Serial.println("[Trim] Valve " + String(index + 1) + ": open " + String(v->getOpenAngle()) + " (trim " + String(v->getOpenPositionTrim()) + ")"
                   + ", closed " + String(v->getClosedAngle()) + " (trim " + String(v->getClosedPositionTrim()) + ")"
                   + (saved ? " [saved]" : " [defaults]")
                   + ((int)index == trimValveIndex_ ? " <- selected" : ""));
}

void Dispenser::printValveTrims() {
    for (uint8_t i = 0; i < valves_.size(); i++) {
        printValveTrim_(i);
    }
}

void Dispenser::loadValveTrims() {
    Preferences prefs;
    if (!prefs.begin(TRIM_NVS_NAMESPACE, true)) {
        Serial.println("[Trim] No saved trims. Using defaults.");
        return;
    }
    for (uint8_t i = 0; i < valves_.size(); i++) {
        String o = "v" + String(i) + "o";
        String c = "v" + String(i) + "c";
        if (prefs.isKey(o.c_str()) && prefs.isKey(c.c_str())) {
            valves_[i]->setTrims(prefs.getInt(o.c_str(), 0), prefs.getInt(c.c_str(), 0));
            valves_[i]->setPosition(Valve::Position::CLOSED); // Apply the calibrated closed angle now.
        }
    }
    prefs.end();
    printValveTrims();
}

bool Dispenser::saveSelectedValveTrim() {
    if (trimValveIndex_ < 0) {
        Serial.println("[Trim] No valve selected. Press 1-6 or q-y first.");
        return false;
    }
    return saveValveTrim(trimValveIndex_ + 1);
}

bool Dispenser::isValveTrimSaved_(uint8_t index) {
    bool saved = false;
    Preferences prefs;
    if (prefs.begin(TRIM_NVS_NAMESPACE, true)) {
        saved = prefs.isKey(("v" + String(index) + "o").c_str());
        prefs.end();
    }
    return saved;
}

uint8_t Dispenser::getSelectedTrimValveId() {
    return trimValveIndex_ < 0 ? 0 : trimValveIndex_ + 1;
}

Valve::Position Dispenser::getValvePosition(uint8_t valveId) {
    if (!isValidValveId_(valveId)) {
        return Valve::Position::CLOSED;
    }
    return valves_[valveId - 1]->getPosition();
}

uint8_t Dispenser::getValveCount() {
    return valves_.size();
}

bool Dispenser::isValidValveId_(uint8_t valveId) {
    return valveId >= 1 && valveId <= valves_.size();
}

bool Dispenser::setValveAngle(uint8_t valveId, Valve::Position position, int32_t angle) {
    if (!isValidValveId_(valveId) || angle < 0 || angle > 180) {
        return false;
    }
    trimValveIndex_ = valveId - 1;
    const auto& v = valves_[trimValveIndex_];
    if (position == Valve::Position::OPEN) {
        v->trimOpen(angle - (int32_t)v->getOpenPosition());     // also moves it open
    } else {
        v->trimClosed(angle - (int32_t)v->getClosedPosition()); // also moves it closed
    }
    printValveTrim_(trimValveIndex_);
    return true;
}

bool Dispenser::saveValveTrim(uint8_t valveId) {
    if (!isValidValveId_(valveId)) {
        return false;
    }
    uint8_t index = valveId - 1;
    Preferences prefs;
    if (!prefs.begin(TRIM_NVS_NAMESPACE, false)) {
        Serial.println("[Trim] ERROR: could not open flash storage.");
        return false;
    }
    const auto& v = valves_[index];
    bool ok = prefs.putInt(("v" + String(index) + "o").c_str(), v->getOpenPositionTrim()) > 0
           && prefs.putInt(("v" + String(index) + "c").c_str(), v->getClosedPositionTrim()) > 0;
    prefs.end();
    Serial.print(ok ? "[Trim] Saved. " : "[Trim] ERROR: save failed. ");
    printValveTrim_(index);
    return ok;
}

bool Dispenser::resetValveTrim(uint8_t valveId) {
    if (!isValidValveId_(valveId)) {
        return false;
    }
    trimValveIndex_ = valveId - 1;
    resetTrimPositions();
    return true;
}

// Calibration status line for the BLE protocol:
// T<id>=<openAngle>,<closedAngle>,<defaultOpen>,<defaultClosed>,<saved 0|1>
std::string Dispenser::valveTrimStatus(uint8_t valveId) {
    if (!isValidValveId_(valveId)) {
        return "";
    }
    const auto& v = valves_[valveId - 1];
    return "T" + std::to_string(valveId) + "=" + std::to_string(v->getOpenAngle()) + ","
         + std::to_string(v->getClosedAngle()) + "," + std::to_string(v->getOpenPosition()) + ","
         + std::to_string(v->getClosedPosition()) + "," + (isValveTrimSaved_(valveId - 1) ? "1" : "0");
}

bool Dispenser::beginDispensingPump(uint8_t pumpIndex, float targetWeight) {
    if (pumpIndex >= pumps_.size()) {
        Serial.println("[Dispenser][beginDispensing] Invalid pump Index: " + String(pumpIndex) + " out of " + String(pumps_.size()) + " pumps.");
        return false;
    }

    dispenseType_ = DispenseType::PUMP;
    state_ = DispenserState::AWAITING_STABILITY;
    Serial.println("[Dispenser][beginDispensing] Beginning dispensing on pump IDX: " + String(pumpIndex));    
    targetWeight_ = targetWeight;
    awaitingStabilityTimeStampMS_ = millis();

    pourDeviceIndex_ = pumpIndex;
    return true;
};

bool Dispenser::beginDispensingValve(uint8_t valveIndex, float targetWeight) {
    if (valveIndex >= valves_.size()) {
        Serial.println("[Dispenser][beginDispensing] Invalid valve Index: " + String(valveIndex) + " out of " + String(valves_.size()) + " valves.");
        return false;
    }

    dispenseType_ = DispenseType::VALVE;
    state_ = DispenserState::AWAITING_STABILITY;
    Serial.println("[Dispenser][beginDispensing] Beginning dispensing on valve IDX: " + String(valveIndex));    
    targetWeight_ = targetWeight;
    awaitingStabilityTimeStampMS_ = millis();
    pourDeviceIndex_ = valveIndex;
    return true;
};

// void Dispenser::beginDispensing(uint8_t valveOrPumpIndex, float targetWeight, DispenseType type) {

//     if (type == DispenseType::PUMP) {        
//         if (valveOrPumpIndex > pumps_.size()) {
//             Serial.println("[Dispenser][beginDispensing] Invalid pump Index: " + String(valveOrPumpIndex) + " out of " + String(pumps_.size()-1) + " pumps.");
//             return;
//         }
//         pumpIndex_ = valveOrPumpIndex-1;       
//     } else {
//         if (valveOrPumpIndex > valves_.size()) {
//             Serial.println("[Dispenser][beginDispensing] Invalid valve Index: " + String(valveOrPumpIndex) + " out of " + String(valves_.size()-1) + " valves.");
//             return;
//         }
//         valveIndex_ = valveOrPumpIndex-1;
//     }    
    
//     dispenseType_ = type;
//     state_ = DispenserState::AWAITING_STABILITY;
//     tare();
//     Serial.println("[Dispenser][beginDispensing] Beginning dispensing on internal IDX: " + String(valveOrPumpIndex));    
//     targetWeight_ = targetWeight;        
//     awaitingStabilityTimeStampMS_ = millis();    
// };

void Dispenser::abortDispensing() {    
    Serial.println("[Dispenser][abortDispensing] Aborting dispensing.");

    if (state_ == DispenserState::READY || state_ == DispenserState::FINISHED) {
        return; // Nothing in progress, no output to switch off.
    }

    if (dispenseType_ == DispenseType::PUMP) {
        pumps_[pourDeviceIndex_]->off();
    } else {
        valves_[pourDeviceIndex_]->setPosition(Valve::Position::CLOSED);
    }
    
    resetDispensing_(true); //Skip Callback....
};

void Dispenser::finishDispensing_() {        
    if (dispenseType_ == DispenseType::PUMP) {
        Serial.println("[Dispenser][finishDispensing_] Finishing dispensing internal pump IDX: " + String(pourDeviceIndex_));
        pumps_[pourDeviceIndex_]->off();
    } else {
        Serial.println("[Dispenser][finishDispensing_] Finishing dispensing internal valve IDX: " + String(pourDeviceIndex_));
        valves_[pourDeviceIndex_]->setPosition(Valve::Position::CLOSED);    
    }

    state_ = DispenserState::AWAITING_CLOSURE;
    awaitingClosureTimeStampMS_ = millis();
};

bool Dispenser::isPouring() {
    return state_ == DispenserState::DISPENSING || state_ == DispenserState::AWAITING_CLOSURE;
}

Dispenser::DispenserState Dispenser::getState() {
    return state_;
};

float Dispenser::getLatestWeight() {
    return latestWeight_ ;
};

float Dispenser::getAbsoluteWeight() {
    float offset = ((float)scale_->get_offset() / (float)scale_->get_scale());
    float absWeight = (latestWeight_ + offset) - emptyWeight_;

    return absWeight; //fabs(offset);
};

void Dispenser::resetDispensing_(bool skipCallback) {    

    uint8_t index = valveIndex_;
    if (dispenseType_ == DispenseType::PUMP) {
        index = pumpIndex_;
    } 

    if (completionCallback_ != nullptr && skipCallback == false) { completionCallback_(dispenseType_, valveIndex_, latestWeight_); }

    state_ = DispenserState::FINISHED;
    Serial.println("[Dispenser][resetDispensing_] Resetting dispensing state.");
    targetWeight_ = 0.0;
    valveIndex_ = 0;
    pumpIndex_ = 0;
    latestWeight_ = 0.0;
    
    awaitingClosureTimeStampMS_ = 0;
};

void Dispenser::setAllValves(Valve::Position position) { 
    for (const auto& valvePtr : valves_) {
        valvePtr->setPosition(position);  
    }    
};

void Dispenser::trimValve(int value) {
    if (trimValveIndex_ < 0) {
        Serial.println("[Trim] No valve selected. Press 1-6 or q-y first.");
        return;
    }
    const auto& valvePtr = valves_[trimValveIndex_];

    Valve::Position position = valvePtr->getPosition();
        

    if (position == Valve::Position::CLOSED) {
        valvePtr->trimClosed(valvePtr->getClosedPositionTrim() + value);
    } else if (position == Valve::Position::OPEN) {
        valvePtr->trimOpen(valvePtr->getOpenPositionTrim() + value);
    }
    printValveTrim_(trimValveIndex_);

};

void Dispenser::resetTrimPositions() {
    if (trimValveIndex_ < 0) {
        Serial.println("[Trim] No valve selected. Press 1-6 or q-y first.");
        return;
    }

    const auto& valvePtr = valves_[trimValveIndex_];
    valvePtr->resetTrimPositions();
    valvePtr->setPosition(valvePtr->getPosition()); // Move to the default angle so the change is visible.
    Serial.print("[Trim] Reset to defaults (not saved, press ! to save). ");
    printValveTrim_(trimValveIndex_);

};

uint8_t Dispenser::registerValve(std::shared_ptr<Valve> valve) {    
    valves_.push_back(valve);
    return valves_.size();
};

uint8_t Dispenser::registerPump(std::shared_ptr<Pump> pump) {
    pumps_.push_back(pump);
    return pumps_.size();
};

void Dispenser::registerCompletionCallback(DispenseCompleteCallback callback) {
    completionCallback_ = callback;
};