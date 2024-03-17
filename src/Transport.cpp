#include "Transport.h"

Transport::Transport(uint8_t PIN_HOME_SW, uint8_t PIN_ENABLE, uint8_t PIN_CS, uint32_t parkStepAddress): 
    PIN_HOME_SW_(PIN_HOME_SW), 
    PIN_ENABLE_(PIN_ENABLE), 
    PIN_CS_(PIN_CS) 
{
  motor_ = std::make_unique<TMC5160_SPI>(PIN_CS);  
  TMC5160::PowerStageParameters powerStageParams; // defaults.
  TMC5160::MotorParameters motorParams;
  motorParams.globalScaler = 60; // Adapt to your driver and motor (check TMC5160 datasheet - "Selecting sense resistors")
  motorParams.irun = 25;
  motorParams.ihold = 17;
  motor_->begin(powerStageParams, motorParams, TMC5160::NORMAL_MOTOR_DIRECTION);  
  motor_->setRampMode(TMC5160::POSITIONING_MODE);
  motor_->setMaxSpeed(400);
  motor_->setAcceleration(250);
  digitalWrite(PIN_ENABLE_, LOW);
  motor_->enable();  
  defineStation(parkStepAddress);

}

Transport::~Transport() {}

void Transport::heartbeat() {
  
  switch (machineState_)
  {
  //write all cases
  case Transport::MachineState::NOT_READY:
    break;    
  case Transport::MachineState::HOMING: 
  if (homingStage_ == Transport::HomingStage::SEEKING_HOME) {
    homing_awaiting_rough_home_sw();
  }
  if (homingStage_ == Transport::HomingStage::RETRACTING) {    
    homing_awaiting_retract();
  }
  if (homingStage_ == Transport::HomingStage::REFINING) {
    homing_awaiting_home_sw_refining();
  }
    break;
  case Transport::MachineState::MOVING_TO_TARGET_POS:
    awaiting_target_pos();
    break;
  case Transport::MachineState::AT_TARGET:
    break;  
  default:
    break;
  }
}


void Transport::moveStepsRight(uint32_t steps) {
  motor_->setTargetPosition(motor_->getCurrentPosition() - steps);
}

void Transport::moveStepsLeft(uint32_t steps) {
  motor_->setTargetPosition(motor_->getCurrentPosition() + steps);
}

uint32_t Transport::defineStation(int32_t stepAddress) {
  Station station;  
  station.stepAddress = stepAddress;
  stations_.push_back(station);
  return stations_.size() - 1;
}

uint32_t Transport::getCurrentPosition() {
  return motor_->getCurrentPosition();
}


void Transport::refMachine(DidHomeCallback didHomeCallback) {  
  Serial.println("[Transport][refMachine] -> Homing...");
  
  if (didHomeCallback != nullptr) {
    didHomeCallback_ = std::make_shared<DidHomeCallback>(didHomeCallback);
  } 
  
  setState_(Transport::MachineState::HOMING);
  homingStage_ = Transport::HomingStage::SEEKING_HOME;
  motor_->stop();  
  motor_->setCurrentPosition(4000);    
  motor_->setMaxSpeed(60);  
  motor_->setTargetPosition(0); 
}

void Transport::homing_awaiting_rough_home_sw() {  
  
  if (digitalRead(PIN_HOME_SW_) == HIGH ) {        
    Serial.println("[Transport][refMachine] -> Rogh home switch triggered. Retracting...");
    motor_->stop();  
    motor_->setCurrentPosition(0);
    motor_->setMaxSpeed(40);  
    motor_->setTargetPosition(40);
    homingStage_ = Transport::HomingStage::RETRACTING;
    return;
  }
}

void Transport::homing_awaiting_retract() {
 if (motor_->getCurrentPosition() >= 40) {  
    Serial.println("[Transport][refMachine] -> Retract position reached. Refining...");
    motor_->setMaxSpeed(20);  
    motor_->setTargetPosition(-10);
    homingStage_ = Transport::HomingStage::REFINING;
    return;
  } 
}

void Transport::homing_awaiting_home_sw_refining() {
  if (digitalRead(PIN_HOME_SW_) == HIGH) {
    Serial.println("[Transport][refMachine] -> Unit is fully HOMED... Parking");
    motor_->stop();  
    motor_->setCurrentPosition(0);    
    goPark(50);

    if (didHomeCallback_ != nullptr) {
      (*didHomeCallback_)(true);
    }

    return;
  }
}

void Transport::setState_(MachineState state) {
    if (machineState_ != state) {
        machineState_ = state;
        if (stateDidChangeCallback_ != nullptr) {
            (*stateDidChangeCallback_)(state);
        }
    }
}


void Transport::goPark(uint16_t speed) {
  motor_->setMaxSpeed(speed);
  goToStation(0, speed);
}

void Transport::goToStation(uint8_t stationIndex, uint16_t speed) {
  if (stationIndex >= stations_.size()) {
    Serial.println("[Transport][goToStation] -> Station index out of range");
    return;
  }
  
  uint32_t targetPos = stations_[stationIndex].stepAddress;
  currentStation_ = std::make_shared<Station>(stations_[stationIndex]);
  currentStationIndex_ = stationIndex;
  setState_(Transport::MachineState::MOVING_TO_TARGET_POS);
  motor_->setMaxSpeed(speed);  
  motor_->setTargetPosition(currentStation_->stepAddress);
}

bool Transport::isAtTarget() {
  if (machineState_ == Transport::MachineState::AT_TARGET) {
    return true;
  }

  return false;
}

void Transport::awaiting_target_pos() {  

  if (motor_->getCurrentPosition() == currentStation_->stepAddress) {
    Serial.println("[Transport][awaiting_target_pos] -> Target position reached: " + String(currentStationIndex_));
    setState_(Transport::MachineState::AT_TARGET);

    if (targetReachedCallback_ != nullptr) {
      (*targetReachedCallback_)(currentStation_, currentStationIndex_);      
    }  
    
    return;
  }
}

bool Transport::isParked() {
  if (machineState_ == Transport::MachineState::AT_TARGET && currentStationIndex_ == 0) {
    return true;
  }
  return false;
}

bool Transport::isReady() {
  if (machineState_ == Transport::MachineState::AT_TARGET) {
    return true;
  }
  
  return false;
}

Transport::MachineState Transport::getState() {
  return machineState_;
}


void Transport::setStateDidChangeCallback(StateChangeCallback callback) {
    stateDidChangeCallback_ = std::make_shared<StateChangeCallback>(callback);
}

void Transport::setTargetReachedCalledBack(TargetReachedCallback callback) {
    targetReachedCallback_ = std::make_shared<TargetReachedCallback>(callback);
}