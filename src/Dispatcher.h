#include "Arduino.h"
#include <Dispenser.h>
#include <Transport.h>
#include <memory>

#pragma once

class Dispatcher
{
public:   
    enum class DispatcherState {
        NO_CUP,
        READY,
        MOVING,
        SERVING,
        AWAITING_END_DELAY,
        AWAITING_START_DELAY,
        STEP_COMPLETE,
        AWAITING_REMOVAL,
        JOB_COMPLETE,
    };

    struct StepStatus
    {
        uint8_t index;
        uint8_t stationIndex;
        float targetWeight;
        float dispensedWeight;
        bool stepCompleted = false;
        DispatcherState state;        
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
    

using WillBeginDispensing = std::function<void(const uint8_t&)>;
using DidFinishDispensing =  std::function<void(const uint8_t&)>;
using DidUpdateWeight =  std::function<void(const uint8_t&, float weight)>;
using DidFinishJob =  std::function<void()>;
using IsReady =  std::function<void()>;

Dispatcher(std::shared_ptr<Dispenser> dispenser, std::shared_ptr<Transport> transport);
void heartbeat();
void clearSteps();
void addStep(Dispenser::DispenseType type, uint8_t index, uint8_t stationIndex, float targetWeight);
bool start();
void cancel();
bool isServing();

void setWillBeginDispensingCallback(WillBeginDispensing callback);
void setDidFinishDispensingCallback(DidFinishDispensing callback);
void setDidUpdateWeight(DidUpdateWeight callback);
void setDidFinishJob(DidFinishJob callback);
void setIsReady(IsReady callback);

DispatcherState getState();
StepStatus getStepStatus();
    
        
private:    
    void movingPhase_();
    void servingPhase_();
    void awaitingEndDelayPhase_();
    void awaitingStartDelayPhase_();
    void awaitingRemovalPhase_();
    void jobCompletePhase_();
    void performNextStep_(); 
    void reset_();

    std::shared_ptr<Dispenser> dispenser_;
    std::shared_ptr<Transport> transport_;

    WillBeginDispensing willBeginDispensingCallback_;
    DidFinishDispensing didFinishDispensingCallback_;
    DidUpdateWeight didUpdateWeightCallback_;
    DidFinishJob didFinishJobCallback_;
    IsReady isReadyCallback_;

    std::vector<Steps> steps_;
    DispatcherState state_;
    uint32_t jobBeginTimeStampMS_;

    uint8_t currentStep_=0;
    float cumulativeWeight_ = 0.0;
    
    
};
