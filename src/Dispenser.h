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

    Dispenser(uint8_t dat_pin, uint8_t sck_pin, float kFactor = 428.0, float emptyWeight = 0.0);
    u_int8_t registerValve(std::shared_ptr<Valve> valve);
    u_int8_t registerPump(std::shared_ptr<Pump> pump);
    
    // void beginDispensing(uint8_t valveOrPumpIndex, float targetWeight, DispenseType type);    
    
    // Return false (and leave the dispenser untouched) if the index is out of range.
    bool beginDispensingPump(uint8_t pourDeviceIndex, float targetWeight);
    bool beginDispensingValve(uint8_t pourDeviceIndex, float targetWeight);

    void registerCompletionCallback(DispenseCompleteCallback callback);
    void abortDispensing();
    void heartbeat();
    void tare();
    DispenserState getState();
    bool isPouring(); // Output open, or closing and settling: weight readings belong to this step.
    float getLatestWeight();
    float getAbsoluteWeight();
    void setAllValves(Valve::Position position);    
    void trimValve(int value);
    void resetTrimPositions();
    void selectValveForTrim(uint32_t valveId, Valve::Position position); // valveId is 1-based
    void loadValveTrims();          // Apply trims saved in flash (call after registering valves).
    bool saveSelectedValveTrim();   // Persist the selected valve's open + closed trims.
    void printValveTrims();
    // Calibration by 1-based valve id (used by BLE). All return false on a bad id/angle.
    uint8_t getValveCount();
    bool setValveAngle(uint8_t valveId, Valve::Position position, int32_t angle); // 0..180, moves the valve
    bool saveValveTrim(uint8_t valveId);
    bool resetValveTrim(uint8_t valveId);   // back to defaults, not saved
    std::string valveTrimStatus(uint8_t valveId);
    uint8_t getSelectedTrimValveId();                    // 1-based, 0 = none selected
    Valve::Position getValvePosition(uint8_t valveId);   // 1-based; CLOSED if invalid
    std::unique_ptr<HX711> scale_;
    

private:
    void resetDispensing_(bool skipCallback = false);
    void finishDispensing_();
    void printValveTrim_(uint8_t index);
    bool isValveTrimSaved_(uint8_t index);
    bool isValidValveId_(uint8_t valveId);
    int trimValveIndex_ = -1; // Valve selected for trimming (0-based), -1 = none. Independent of pouring.
    DispenseCompleteCallback completionCallback_;
    std::vector<std::shared_ptr<Valve>> valves_;
    std::vector<std::shared_ptr<Pump>> pumps_;
    
    float targetWeight_;
    float latestWeight_;
    uint8_t valveIndex_;    
    uint8_t pumpIndex_;
    uint8_t pourDeviceIndex_;

    float emptyWeight_;    
    float filteredValue_;
    DispenseType dispenseType_;
    uint32_t awaitingClosureTimeStampMS_;
    uint32_t awaitingStabilityTimeStampMS_;
    DispenserState state_;    
    
    
};
