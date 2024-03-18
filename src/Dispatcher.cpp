#include "Dispatcher.h"


void Dispatcher::heartbeat() {
    switch (state_)
    {
    case DispatcherState::READY:
        break;
    case DispatcherState::MOVING:
        movingPhase_();
        break;
    case DispatcherState::SERVING:
        servingPhase_();
        break;
    case DispatcherState::AWAITING_END_DELAY:
        awaitingEndDelayPhase_();
        break;            
    case DispatcherState::STEP_COMPLETE:
        cumulativeWeight_ += dispenser_->getLatestWeight();
        break;
    case DispatcherState::JOB_COMPLETE:
        jobCompletePhase_();
        break;
    default:
        break;
    }
    
}

void Dispatcher::movingPhase_() {
    if (transport_->getState() == Transport::MachineState::AT_TARGET) {
        Serial.println("[Dispatcher][movingPhase_] Transport at target.");
        state_ = DispatcherState::SERVING;
        steps_[currentStep_].beginDispensingTimeStampMS = millis();
        dispenser_->beginDispensing(steps_[currentStep_].index, steps_[currentStep_].targetWeight, steps_[currentStep_].type);
    }    
}

void Dispatcher::servingPhase_() {
    if (dispenser_->getState() == Dispenser::DispenserState::READY) {
        Serial.println("[Dispatcher][servingPhase_] Dispensing Complete.");
        state_ = DispatcherState::AWAITING_END_DELAY;
        steps_[currentStep_].endDispensingTimeStampMS = millis();
    }
}

void Dispatcher::awaitingEndDelayPhase_() {
    if (millis() - steps_[currentStep_].endDispensingTimeStampMS > 200) {
        Serial.println("[Dispatcher][awaitingDelayPhase_] Delay complete.");
        state_ = DispatcherState::STEP_COMPLETE;
        steps_[currentStep_].stepCompleted = true;
        currentStep_++;
        if (currentStep_ >= steps_.size()) {
            Serial.println("[Dispatcher][awaitingDelayPhase_] All steps complete.");
            state_ = DispatcherState::JOB_COMPLETE;
            transport_->goPark();
        }
        else {
           performNextStep_();
        }
    }
}

void Dispatcher::jobCompletePhase_() {    
    if (transport_->isParked()) {
        Serial.println("[Dispatcher][jobCompletePhase_] Job is complete. Clearing Recepie.");
        reset_();
    }
}

void Dispatcher::cancel() {
    Serial.println("[Dispatcher][cancel] Cancelling job.");
    dispenser_->abortDispensing();
    transport_->goPark();
    state_ = DispatcherState::JOB_COMPLETE;
}

void Dispatcher::reset_() {
        Serial.println("[Dispatcher][reset_] Job complete: " + String(steps_.size()) + " steps executed in " + String(millis() - jobBeginTimeStampMS_) + "ms.");
        state_ = DispatcherState::READY;
        steps_.clear();
        currentStep_ = 0;
        cumulativeWeight_ = 0.0;
        jobBeginTimeStampMS_ = 0;
}

void Dispatcher::performNextStep_() {
        Serial.println("[Dispatcher][performNextStep_] Performing next step: " + String(currentStep_) + " of " + String(steps_.size()-1));
        transport_->goToStation(steps_[currentStep_].index);
        steps_[currentStep_].beginMovementTimeStampMS = millis();        
        state_ = DispatcherState::MOVING;
}

void Dispatcher::clearSteps() {
    steps_.clear();
}

void Dispatcher::addStep(Dispenser::DispenseType type,  uint8_t index, uint8_t stationIndex, float targetWeight) {
    Steps step;
    step.index = index;
    step.stationIndex = stationIndex;
    step.type = type;
    step.targetWeight = targetWeight;
    step.stepCompleted = false;
    steps_.push_back(step);
}

bool Dispatcher::start() {
    if (steps_.size() == 0) {
        Serial.println("[Dispatcher][start] No steps to execute.");
        return false;
    }

    if (transport_->isParked() == false) {
        Serial.println("[Dispatcher][start] Transport not ready.");
        return false;
    }    

    currentStep_ = 0;
    cumulativeWeight_ = 0.0;
    jobBeginTimeStampMS_ = millis();    
    
    Serial.println("[Dispatcher][start] Performing first step: " + String(currentStep_) + " of " + String(steps_.size()-1));
    Serial.println("[Dispatcher][start] Start weight: " + String(dispenser_->getLatestWeight()) + "g");

    dispenser_->tare();
    transport_->goToStation(steps_[currentStep_].stationIndex);
    steps_[currentStep_].beginMovementTimeStampMS = millis();
    state_ = DispatcherState::MOVING;

    return true;
}

Dispatcher::DispatcherState Dispatcher::getState() {
    return state_;
}

Dispatcher::Dispatcher(std::shared_ptr<Dispenser> dispenser, std::shared_ptr<Transport> transport) {
    dispenser_ = dispenser;
    transport_ = transport;
    state_ = DispatcherState::READY;
};

