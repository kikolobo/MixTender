#include "Arduino.h"
#include <Dispenser.h>
#include <Transport.h>
#include <memory>

#pragma once

class Dispatcher
{
public:   
    enum class DispatcherState {
        READY,
        MOVING,
        SERVING,
        AWAITING_END_DELAY,
        AWAITING_START_DELAY,
        STEP_COMPLETE,
        JOB_COMPLETE,
    };


    struct Steps
    {
        uint8_t index;
        uint8_t stationIndex;
        float targetWeight;
        uint32_t beginMovementTimeStampMS;
        uint32_t beginDispensingTimeStampMS;
        u_int32_t endDispensingTimeStampMS;
        bool stepCompleted = false;
        Dispenser::DispenseType type;
    };
    


Dispatcher(std::shared_ptr<Dispenser> dispenser, std::shared_ptr<Transport> transport);
void heartbeat();
void clearSteps();
void addStep(Dispenser::DispenseType type, uint8_t index, uint8_t stationIndex, float targetWeight);
bool start();
void cancel();
DispatcherState getState();
    
        
private:    
    void movingPhase_();
    void servingPhase_();
    void awaitingEndDelayPhase_();
    void awaitingStartDelayPhase_();
    void jobCompletePhase_();
    void performNextStep_(); 
    void reset_();

    std::shared_ptr<Dispenser> dispenser_;
    std::shared_ptr<Transport> transport_;

    std::vector<Steps> steps_;
    DispatcherState state_;
    uint32_t jobBeginTimeStampMS_;

    uint8_t currentStep_=0;
    float cumulativeWeight_ = 0.0;
    
    
};
