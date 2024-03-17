#include <TMC5160.h>
#include "Arduino.h"
#include <memory>
#include <functional>
#include <vector>

#pragma once

class Transport
{
public:

    struct Station {        
        int32_t stepAddress;
    };

    enum class MachineState {
        NOT_READY,          
        HOMING,                
        MOVING_TO_TARGET_POS,
        AT_TARGET,
    };

      enum class HomingStage {
        NONE,
        SEEKING_HOME,           
        RETRACTING,
        REFINING,
        PARKED,
    };

    using StateChangeCallback = std::function<void(const MachineState&)>;
    using DidHomeCallback = std::function<void(bool success)>;
    using TargetReachedCallback = std::function<void(std::shared_ptr<Station>, uint8_t stationIdx)>;

    Transport(uint8_t PIN_HOME_SW = 32, uint8_t PIN_ENABLE = RX, uint8_t PIN_CS = TX, uint32_t parkStepAddress = 15);
    ~Transport();
    void heartbeat();
    void refMachine(DidHomeCallback didHome = nullptr);    
    void setStateDidChangeCallback(StateChangeCallback callback);     
    void setTargetReachedCalledBack(TargetReachedCallback callback);     
    uint32_t defineStation(int32_t stepAddress);
    void goToStation(uint8_t stationIndex, uint16_t speed = 500);
    uint32_t getCurrentPosition();
    void moveStepsRight(u_int32_t steps = 0);
    void moveStepsLeft(u_int32_t steps = 0);
    void goPark(uint16_t speed = 500);
    bool isParked();
    bool isAtTarget();
    bool isReady();
    MachineState getState();

private:   
    uint8_t PIN_HOME_SW_;
    uint8_t PIN_ENABLE_;
    uint8_t PIN_CS_;

    std::vector<Station> stations_;
    std::shared_ptr<Station> currentStation_;
    int8_t currentStationIndex_ = -1;

    std::shared_ptr<StateChangeCallback> stateDidChangeCallback_;
    std::shared_ptr<DidHomeCallback> didHomeCallback_;
    std::shared_ptr<TargetReachedCallback> targetReachedCallback_;
    
    MachineState machineState_ = MachineState::NOT_READY;
    HomingStage homingStage_ = HomingStage::NONE;

    std::unique_ptr<TMC5160_SPI> motor_;   


    void setState_(MachineState state);
    void referencingMachine_();

    void homing_awaiting_rough_home_sw();
    void homing_awaiting_retract();
    void homing_awaiting_home_sw_refining();
    void awaiting_target_pos();    
    
};
