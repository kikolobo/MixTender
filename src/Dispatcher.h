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
        UNKNOWN,
    };

    enum class AbortReason {
        CUP_REMOVED,
        NO_FLOW,
        POUR_TIMEOUT,
        SCALE,
        DISCONNECTED,   // App disconnected mid-job
    };

    static const uint32_t WEIGHT_REPORT_INTERVAL_MS = 100; // W<n> updates at most ~10 Hz

    static constexpr float CUP_REMOVED_GRAMS = 3.0;      // Tray clear: cup lifted mid-job
    static constexpr float DEFAULT_MIN_CUP_GRAMS = 10.0; // Cup detected above this (settable)
    static constexpr float CUP_DETECT_HYSTERESIS_GRAMS = 10.0;
    static const uint32_t CUP_REMOVED_DEBOUNCE_MS = 300;

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
        uint8_t stationIndex;
        uint8_t pourDeviceIndex;        
        float targetWeight;
        uint32_t beginMovementTimeStampMS;
        uint32_t beginDispensingTimeStampMS;
        u_int32_t endDispensingTimeStampMS;
        bool stepCompleted = false;
        Dispenser::DispenseType type;
    };


using WillBeginDispensing = std::function<void(const uint8_t&)>;
using DidFinishDispensing =  std::function<void(const uint8_t&)>;
using DidFailDispensing =  std::function<void(const uint8_t&)>;
using DidAbortJob =  std::function<void(AbortReason)>;
using DidUpdateWeight =  std::function<void(const uint8_t&, float weight)>;
using DidFinishJob =  std::function<void()>;
using IsReady =  std::function<void()>;

Dispatcher(std::shared_ptr<Dispenser> dispenser, std::shared_ptr<Transport> transport);
void heartbeat();
void clearSteps();
void addStep(Dispenser::DispenseType type,  uint8_t stationIndex, uint8_t pourDeviceIndex, float targetWeight);
bool start();
bool cancel(); // false if no job was in progress
bool abort(AbortReason reason); // Safety stop from outside; false if no job was in progress
bool isServing();

void setWillBeginDispensingCallback(WillBeginDispensing callback);
void setDidFinishDispensingCallback(DidFinishDispensing callback);
void setDidFailDispensingCallback(DidFailDispensing callback);
void setDidAbortJobCallback(DidAbortJob callback);
void setDidUpdateWeight(DidUpdateWeight callback);
void setDidFinishJob(DidFinishJob callback);
void setIsReady(IsReady callback);

DispatcherState getState();
StepStatus getStepStatus();
// Lightest empty cup: a cup is detected above this weight (over the empty tray) and
// lost at max(CUP_REMOVED_GRAMS, minCup - CUP_DETECT_HYSTERESIS_GRAMS).
void setMinCupGrams(float grams);
float getMinCupGrams();
    
        
private:
    float minCupGrams_ = DEFAULT_MIN_CUP_GRAMS;    
    void movingPhase_();
    void servingPhase_();
    void awaitingEndDelayPhase_();
    void awaitingStartDelayPhase_();
    void awaitingRemovalPhase_();
    void jobCompletePhase_();
    void performNextStep_();
    void advanceStep_();
    void abortJob_(AbortReason reason);
    uint32_t cupMissingSinceMs_ = 0;
    uint32_t lastWeightReportMs_ = 0;
    void reset_();

    std::shared_ptr<Dispenser> dispenser_;
    std::shared_ptr<Transport> transport_;

    WillBeginDispensing willBeginDispensingCallback_;
    DidFinishDispensing didFinishDispensingCallback_;
    DidFailDispensing didFailDispensingCallback_;
    DidAbortJob didAbortJobCallback_;
    DidUpdateWeight didUpdateWeightCallback_;
    DidFinishJob didFinishJobCallback_;
    IsReady isReadyCallback_;

    std::vector<Steps> steps_;
    DispatcherState state_;
    uint32_t jobBeginTimeStampMS_;

    uint8_t currentStep_=0;
    float cumulativeWeight_ = 0.0;
    
    
};
