#include "Dispenser.h"



Dispenser::Dispenser(uint8_t dat_pin, uint8_t sck_pin, float emptyWeight, float kFactor) {
    scale_ = std::make_unique<HX711>();
    scale_->begin(dat_pin, sck_pin);
    scale_->set_scale(kFactor);
    valves_.clear();
    emptyWeight_ = emptyWeight;    
    scale_->tare();
    valveIndex_ = 0;
    pumpIndex_ = 0;
};


void Dispenser::heartbeat() {
    
    if (scale_->is_ready()) {
        float alpha = 0.4; // Smoothing factor. Adjust as needed.
        float rawValue = fabs(scale_->get_units(1)); // Get the current raw value        
        filteredValue_ = alpha * rawValue + (1 - alpha) * filteredValue_; // Apply the low-pass filter

        float resolution = 0.14; // Target resolution, e.g., 0.1 kg
        float finalValue = round(filteredValue_ / resolution) * resolution; // Reduce resolution


        latestWeight_ = finalValue; //fabs(scale_->get_units(5));
        // Serial.println("[Dispenser][heartbeat] Latest weight: " + String(latestWeight_) + "g");
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
            state_ = DispenserState::STABLE;
            tare();        
        }
    }

    if (state_ == DispenserState::STABLE) {
            Serial.println("[Dispenser][AWAITING_STABILITY] Awaiting stability reached..");            
            Serial.println("[Dispenser][AWAITING_STABILITY] Station Begin weight: " + String(getLatestWeight()) + "g");
            state_ = DispenserState::DISPENSING;
            if (dispenseType_ == DispenseType::PUMP) {
                pumps_[pumpIndex_]->on();
            } else {
                valves_[valveIndex_]->setPosition(Valve::Position::OPEN);                            
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
    scale_->tare();
};


void Dispenser::selectValveForTrim(uint32_t valveId, Valve::Position position) {
    if (valveId > valves_.size()) {
        Serial.println("[Dispenser][selectValveForTrim] Invalid valve Index: " + String(valveId) + " out of " + String(valves_.size()) + " valves.");
        return;
    }
    

    valveIndex_ = valveId-1;

    if (position == Valve::Position::CLOSED) {
        valves_[valveIndex_]->trimClosed(0);
    } else if (position == Valve::Position::OPEN) {
        valves_[valveIndex_]->trimOpen(0);
    }
};

void Dispenser::beginDispensing(uint8_t valveOrPumpIndex, float targetWeight, DispenseType type) {

    if (type == DispenseType::PUMP) {        
        if (valveOrPumpIndex > pumps_.size()-1) {
            Serial.println("[Dispenser][beginDispensing] Invalid pump Index: " + String(valveOrPumpIndex) + " out of " + String(pumps_.size()-1) + " pumps.");
            return;
        }
        pumpIndex_ = valveOrPumpIndex-1;       
    } else {
        if (valveOrPumpIndex > valves_.size()-1) {
            Serial.println("[Dispenser][beginDispensing] Invalid valve Index: " + String(valveIndex_) + " out of " + String(valves_.size()-1) + " valves.");
            return;
        }
        valveIndex_ = valveOrPumpIndex-1;
    }    
    
    dispenseType_ = type;
    state_ = DispenserState::AWAITING_STABILITY;
    
    Serial.println("[Dispenser][beginDispensing] Beginning dispensing on internal IDX: " + String(valveOrPumpIndex));    
    targetWeight_ = targetWeight;        
    awaitingStabilityTimeStampMS_ = millis();    
};

void Dispenser::abortDispensing() {    
    Serial.println("[Dispenser][abortDispensing] Aborting dispensing.");
    
    if (dispenseType_ == DispenseType::PUMP) {
        pumps_[pumpIndex_]->off();
    } else {
        valves_[valveIndex_]->setPosition(Valve::Position::CLOSED);
    }
    
    resetDispensing_(true); //Skip Callback....
};

void Dispenser::finishDispensing_() {        
    if (dispenseType_ == DispenseType::PUMP) {
        Serial.println("[Dispenser][finishDispensing_] Finishing dispensing internal pump IDX: " + String(pumpIndex_));
        pumps_[pumpIndex_]->off();
    } else {
        Serial.println("[Dispenser][finishDispensing_] Finishing dispensing internal valve IDX: " + String(valveIndex_));
        valves_[valveIndex_]->setPosition(Valve::Position::CLOSED);    
    }

    state_ = DispenserState::AWAITING_CLOSURE;
    awaitingClosureTimeStampMS_ = millis();
};

Dispenser::DispenserState Dispenser::getState() {
    return state_;
};

float Dispenser::getLatestWeight() {
    return latestWeight_ ;
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
    const auto& valvePtr = valves_[valveIndex_];

    if (valvePtr == nullptr) {
        Serial.println("[Dispenser][trimValve] Valve Index: " + String(valveIndex_) + " is null.");
        return;
    }

    Valve::Position position = valvePtr->getPosition();
        

    if (position == Valve::Position::CLOSED) {
        valvePtr->trimClosed(valvePtr->getClosedPositionTrim() + value);
        return;
    } else if (position == Valve::Position::OPEN) {
        valvePtr->trimOpen(valvePtr->getOpenPositionTrim() + value);
        return;
    }
    
};

void Dispenser::resetTrimPositions() {
    if (valves_.size() == 0 || valveIndex_ > valves_.size()-1) {
        Serial.println("[Dispenser][resetTrimPositions] No valves registered or selected.");
        return;
    }


    const auto& valvePtr = valves_[valveIndex_];
    valvePtr->resetTrimPositions();
         
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