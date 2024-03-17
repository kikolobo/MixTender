#include "Arduino.h"
#include <Valve.h>
#include <memory>
#include "HX711.h"

#pragma once

class Dispenser
{
public: 

    enum class DispenserState {
        READY,
        DISPENSING,
        AWAITING_CLOSURE,
        AWAITING_STABILITY,
    };

    using DispenseCompleteCallback = std::function<void(uint8_t valveIndex, float dispensedWeight)>;

    Dispenser(uint8_t dat_pin, uint8_t sck_pin, float emptyWeight = 0.0, float kFactor = -438);
    void beginDispensing(uint8_t valveIndex, float targetWeight);    
    u_int8_t registerValve(std::shared_ptr<Valve> valve);
    void registerCompletionCallback(DispenseCompleteCallback callback);
    void abortDispensing();
    void heartbeat();
    void tare();
    DispenserState getState();
    float getLatestWeight();
    void setAllValves(Valve::Position position);    
    void trimValve(int value);
    void resetTrimPositions();
    void selectValveForTrim(uint32_t valveId, Valve::Position position);

private:
    void resetDispensing_(bool skipCallback = false);
    void finishDispensing_();
    DispenseCompleteCallback completionCallback_;
    std::vector<std::shared_ptr<Valve>> valves_;
    std::unique_ptr<HX711> scale_;
    float targetWeight_;
    float latestWeight_;
    uint8_t valveIndex_;
    float emptyWeight_;    
    float filteredValue_;
    uint32_t awaitingClosureTimeStampMS_;
    uint32_t awaitingStabilityTimeStampMS_;
    DispenserState state_;    
    
    
};
