#include "Arduino.h"
#include "Valve.h"
#include "Pump.h"
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
        STABLE,
        FINISHED,
    };

    enum class DispenseType {
        VALVE,
        PUMP,
    };

    using DispenseCompleteCallback = std::function<void(DispenseType type, uint8_t index, float dispensedWeight)>;

    Dispenser(uint8_t dat_pin, uint8_t sck_pin, float emptyWeight = 0.0, float kFactor = -438);
    u_int8_t registerValve(std::shared_ptr<Valve> valve);
    u_int8_t registerPump(std::shared_ptr<Pump> pump);
    
    void beginDispensing(uint8_t valveOrPumpIndex, float targetWeight, DispenseType type);    

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
    std::vector<std::shared_ptr<Pump>> pumps_;
    std::unique_ptr<HX711> scale_;
    float targetWeight_;
    float latestWeight_;
    uint8_t valveIndex_;
    uint8_t pumpIndex_;
    float emptyWeight_;    
    float filteredValue_;
    DispenseType dispenseType_;
    uint32_t awaitingClosureTimeStampMS_;
    uint32_t awaitingStabilityTimeStampMS_;
    DispenserState state_;    
    
    
};
