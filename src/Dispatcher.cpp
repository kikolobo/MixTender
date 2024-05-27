#include "Dispatcher.h"


void Dispatcher::heartbeat() {
    
    switch (state_)
    {
    case DispatcherState::NO_CUP:
        if (dispenser_->getAbsoluteWeight() > 10.0) {
            state_ = DispatcherState::READY;        
        }
        break;
    case DispatcherState::READY:
       if (dispenser_->getAbsoluteWeight() <= 3.0) {
            state_ = DispatcherState::NO_CUP;
        }
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
    case DispatcherState::AWAITING_REMOVAL:
        awaitingRemovalPhase_();
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

        if (steps_[currentStep_].type == Dispenser::DispenseType::PUMP) {
            dispenser_->beginDispensingPump(steps_[currentStep_].pourDeviceIndex, steps_[currentStep_].targetWeight);
        } else {
            dispenser_->beginDispensingValve(steps_[currentStep_].pourDeviceIndex, steps_[currentStep_].targetWeight);
        }
    }    
}

void Dispatcher::servingPhase_() {
    if (dispenser_->getState() == Dispenser::DispenserState::FINISHED) {
        Serial.println("[Dispatcher][servingPhase_] Dispensing Complete.");
        state_ = DispatcherState::AWAITING_END_DELAY;
        steps_[currentStep_].endDispensingTimeStampMS = millis();
    }

    if (didUpdateWeightCallback_) {
            didUpdateWeightCallback_(currentStep_, dispenser_->getLatestWeight());
    }
    
}

void Dispatcher::awaitingEndDelayPhase_() {
    if (millis() - steps_[currentStep_].endDispensingTimeStampMS > 200) {
        Serial.println("[Dispatcher][awaitingDelayPhase_] Delay complete.");
        state_ = DispatcherState::STEP_COMPLETE;
        steps_[currentStep_].stepCompleted = true;
         if (didFinishDispensingCallback_) {
            didFinishDispensingCallback_(currentStep_);
        }
        currentStep_++;
        if (currentStep_ >= steps_.size()) {
            Serial.println("[Dispatcher][awaitingDelayPhase_] All steps complete.");            
            state_ = DispatcherState::AWAITING_REMOVAL;            
            transport_->goPark(); 
            if (didFinishJobCallback_) {
                didFinishJobCallback_();
            }           
        }  else {
           performNextStep_();
        }
    }
}

void Dispatcher::awaitingRemovalPhase_() {
    if (transport_->isParked()) {
        if (dispenser_->getAbsoluteWeight() < 2.0) {
            Serial.println("[Dispatcher][awaitingRemovalPhase_] Cup Removed. Job complete.");            
            state_ = DispatcherState::JOB_COMPLETE;
            if (isReadyCallback_) {
                isReadyCallback_();
            }
            return;
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
        steps_.clear();
        state_ = DispatcherState::NO_CUP;
        currentStep_ = 0;
        cumulativeWeight_ = 0.0;
        jobBeginTimeStampMS_ = 0;
}

void Dispatcher::performNextStep_() {
        Serial.println("[Dispatcher][performNextStep_] Performing next step: " + String(currentStep_) + " of " + String(steps_.size()-1));
        transport_->goToStation(steps_[currentStep_].stationIndex);
        steps_[currentStep_].beginMovementTimeStampMS = millis();        
        state_ = DispatcherState::MOVING;
        if (willBeginDispensingCallback_) {
            willBeginDispensingCallback_(currentStep_);
        }
}

void Dispatcher::clearSteps() {
    steps_.clear();
}

void Dispatcher::addStep(Dispenser::DispenseType type,  uint8_t stationIndex, uint8_t pourDeviceIndex, float targetWeight) {
    Steps step;
    step.stationIndex = stationIndex;    
    step.type = type;
    step.targetWeight = targetWeight;
    step.stepCompleted = false;    
    step.pourDeviceIndex = pourDeviceIndex-1; //One based index to standardize with StationIndex (0==home)
    
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
    
    transport_->goToStation(steps_[currentStep_].stationIndex);
    steps_[currentStep_].beginMovementTimeStampMS = millis();
    state_ = DispatcherState::MOVING;

    return true;
}

Dispatcher::DispatcherState Dispatcher::getState() {
    return state_;
}

Dispatcher::StepStatus Dispatcher::getStepStatus() {
    StepStatus status;
    status.index = currentStep_;
    status.stationIndex = steps_[currentStep_].stationIndex;
    status.targetWeight = steps_[currentStep_].targetWeight;
    status.dispensedWeight = dispenser_->getLatestWeight();
    status.state = state_;
    status.stepCompleted = steps_[currentStep_].stepCompleted;
    return status;
}

bool Dispatcher::isServing() {
    if (state_ == DispatcherState::NO_CUP || state_ == DispatcherState::READY || state_ == DispatcherState::JOB_COMPLETE || state_ == DispatcherState::AWAITING_REMOVAL) {
        return false;
    }
    return true;
}

Dispatcher::Dispatcher(std::shared_ptr<Dispenser> dispenser, std::shared_ptr<Transport> transport) {
    dispenser_ = dispenser;
    transport_ = transport;
    state_ = DispatcherState::READY;
};

void Dispatcher::setWillBeginDispensingCallback(WillBeginDispensing callback) {
    willBeginDispensingCallback_ = callback;
}

void Dispatcher::setDidFinishDispensingCallback(DidFinishDispensing callback) {
    didFinishDispensingCallback_ = callback;
}

void Dispatcher::setDidUpdateWeight(DidUpdateWeight callback) {
    didUpdateWeightCallback_ = callback;
}

void Dispatcher::setDidFinishJob(DidFinishJob callback) {
    didFinishJobCallback_ = callback;
}

void Dispatcher::setIsReady(IsReady callback) {
    isReadyCallback_ = callback;
}

