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
        TARING,      // Non-blocking tare: averaging TARE_SAMPLES readings
        STABLE,
        FINISHED,
        FAILED,      // Pour stopped by a safety check; see getFailureReason()
    };

    enum class FailureReason {
        NONE,
        SCALE,         // Scale stopped answering (tare or pour)
        NO_FLOW,       // Weight not rising: empty bottle / blocked valve
        POUR_TIMEOUT,  // Pour exceeded POUR_MAX_MS
    };

    // Safety limits (ms / g).
    static const uint8_t  TARE_SAMPLES = 10;
    static const uint32_t TARE_TIMEOUT_MS = 3000;     // 10 samples take ~1 s at 10 SPS
    static const uint32_t SCALE_SILENT_MS = 1000;     // No reading for this long = scale fault
    static const uint32_t NO_FLOW_TIMEOUT_MS = 5000;  // Must gain NO_FLOW_MIN_GRAMS within this
    static constexpr float NO_FLOW_MIN_GRAMS = 1.0;
    static const uint32_t POUR_MAX_MS = 90000;        // Hard cap per step

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
    bool tare();                 // Non-blocking; false if a pour is in progress
    bool isScaleResponding();    // A reading arrived within SCALE_SILENT_MS
    FailureReason getFailureReason();
    float getSampleRate();       // Measured HX711 samples/s, 0 until known (~2 s after boot)
    float getLastPourWeight();   // Final weight of the last completed step
    DispenserState getState();
    bool isPouring(); // Output open, or closing and settling: weight readings belong to this step.
    float getLatestWeight();
    float getAbsoluteWeight();   // Weight above the empty tray (gross - empty-tray weight)
    float getGrossWeight();      // Untared reading: scale units since power-up zero, tray included
    // Empty-tray weight: subtracted from the gross reading to get the absolute weight
    // (cup detection, "cup is not empty", cup-removed abort). Saved in flash.
    float getEmptyWeight();
    float getDefaultEmptyWeight();
    bool isEmptyWeightSaved();
    void loadEmptyWeight();              // Apply the value saved in flash, if any (call once in setup).
    bool setEmptyWeight(float grams);    // Set and save. False if out of range or the save failed.
    bool resetEmptyWeight();             // Back to the default and erase the saved value.
    static constexpr float MAX_EMPTY_WEIGHT_GRAMS = 20000.0; // Accepted range: +/- this
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
    void startTare_(DispenserState afterTare);
    void fail_(FailureReason reason);
    FailureReason failureReason_ = FailureReason::NONE;
    DispenserState afterTareState_ = DispenserState::FINISHED;
    int64_t tareSum_ = 0;
    uint8_t tareCount_ = 0;
    uint32_t tareStartMs_ = 0;
    uint32_t lastSampleMs_ = 0;
    uint32_t pourStartMs_ = 0;
    uint32_t lastProgressMs_ = 0;
    float progressWeight_ = 0.0;
    float lastPourWeight_ = 0.0;
    void measureSampleRate_();
    uint32_t rateWindowStartMs_ = 0;
    uint32_t rateSampleCount_ = 0;
    float measuredSps_ = 0.0;
    bool rateReported_ = false;
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
    float defaultEmptyWeight_;
    float filteredValue_;
    DispenseType dispenseType_;
    uint32_t awaitingClosureTimeStampMS_;
    uint32_t awaitingStabilityTimeStampMS_;
    DispenserState state_;    
    
    
};
