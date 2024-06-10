#include <iostream>
#include <vector>
#include <cstring>
#include <Arduino.h>

#include "SPI.h"
#include <TMC5160.h>
#include "HX711.h"
#include "Transport.h"
#include "Valve.h"
#include "Dispenser.h"
#include "Dispatcher.h"
#include "LedManager.h"
#include "BluetoothEngine.h"


#define HOME_SW_PIN 37
#define CS_PIN  TX
#define EN_PIN  RX
#define POCI_MISO_PIN MISO  //CIPO
#define PICO_MOSI_PIN MOSI  //COPI
#define SCK_PIN SCK
#define SERVO5_PIN A5 
#define AX1 A4 
#define AX2 A3
#define LC_DAT A2
#define SERVO0_PIN A0 
#define SERVO1_PIN A1 
#define SERVO2_PIN 14 
#define SERVO3_PIN 32 
#define CH1_PIN 15
#define CH2_PIN 33
#define CH3_PIN 27
#define LC_SCK 12 
#define SERVO4_PIN LED_BUILTIN  //13
const float calibration_factor = 439; 

// HX711 scale;
uint8_t stationIdx = 0;

uint32_t servoAdjustValue_ = 0;
bool machineIsBooted = false;

static unsigned long t_dirchange, lc_update, t_echo;

std::shared_ptr<Transport> transport;
std::shared_ptr<Dispenser> dispenser;
std::unique_ptr<Dispatcher> dispatcher;
std::unique_ptr<LedManager> ledMan;

BluetoothEngine *ble;


std::string rxdData;
bool didReceiveData = false;
bool isConnected = false;
Dispatcher::DispatcherState lastState = Dispatcher::DispatcherState::UNKNOWN;

void handleBleRequests();
void handleSerialRequests();
bool parseBleRequestToDispatcher(const std::string& rxdData);

//Callbacks from dispatcher (prototypes)
void willBeginDispensing(uint8_t step);
void didFinishDispensing(uint8_t step);
void didUpdateWeight(uint8_t step, float weight);
void didFinishJob();
void isReady();
void updateCupState();

void setup() {
  
  Serial.begin(115200);
  
  Serial.println("[BOOT]");
  
  pinMode(CS_PIN, OUTPUT);
  pinMode(EN_PIN, OUTPUT);
  pinMode(SCK_PIN, OUTPUT);
  pinMode(POCI_MISO_PIN, INPUT);
  pinMode(PICO_MOSI_PIN, OUTPUT);
  pinMode(POCI_MISO_PIN, INPUT);
  pinMode(HOME_SW_PIN, INPUT_PULLUP);
  pinMode(SERVO0_PIN, OUTPUT);
  pinMode(SERVO1_PIN, OUTPUT);
  pinMode(SERVO2_PIN, OUTPUT);
  pinMode(SERVO3_PIN, OUTPUT);
  pinMode(SERVO4_PIN, OUTPUT);
  pinMode(SERVO5_PIN, OUTPUT);
  pinMode(CH1_PIN, OUTPUT); 
  pinMode(CH2_PIN, OUTPUT); 
  pinMode(CH3_PIN, OUTPUT); 
  pinMode(SDA, OUTPUT); //INPUT ONLY!
  pinMode(AX1, INPUT); //INPUT ONLY!
  pinMode(AX2, INPUT); //INPUT ONLY!
  
  digitalWrite(CH1_PIN, LOW);
  digitalWrite(CH2_PIN, LOW);
  digitalWrite(CH3_PIN, LOW);
  
 
  Serial.println("# LoboLabs MixTender V1.0 - Dec 2023");
  Serial.println("[main][setup] Initializing System...");

  SPI.begin(SCK_PIN, POCI_MISO_PIN, PICO_MOSI_PIN, CS_PIN);

  transport = std::make_shared<Transport>(HOME_SW_PIN, EN_PIN, CS_PIN);
  dispenser = std::make_shared<Dispenser>(LC_DAT, LC_SCK, calibration_factor , 121.38);
  dispatcher = std::make_unique<Dispatcher>(dispenser, transport);

  Serial.println("[INITIALIZING TRASNPORT]");
  transport->defineStation(12);  
  transport->defineStation(426);
  transport->defineStation(875);
  transport->defineStation(1309);
  transport->defineStation(1759);
  transport->defineStation(2192);
  transport->defineStation(230); //Pumps
  
  //Register Valves
  Serial.println("[INITIALIZING DISPENSER]");
  dispenser->registerValve(std::make_shared<Valve>(SERVO0_PIN));
  dispenser->registerValve(std::make_shared<Valve>(SERVO4_PIN));
  dispenser->registerValve(std::make_shared<Valve>(SERVO2_PIN));
  dispenser->registerValve(std::make_shared<Valve>(SERVO3_PIN));
  dispenser->registerValve(std::make_shared<Valve>(SERVO1_PIN));
  dispenser->registerValve(std::make_shared<Valve>(SERVO5_PIN));

  //Register Pumps
  dispenser->registerPump(std::make_shared<Pump>(CH1_PIN));
  dispenser->registerPump(std::make_shared<Pump>(CH2_PIN));
  dispenser->registerPump(std::make_shared<Pump>(CH3_PIN));

  Serial.println("[INITIALIZING DISPATCHER]");
  dispatcher->setWillBeginDispensingCallback(willBeginDispensing);
  dispatcher->setDidFinishDispensingCallback(didFinishDispensing);
  dispatcher->setDidUpdateWeight(didUpdateWeight);
  dispatcher->setDidFinishJob(didFinishJob);
  dispatcher->setIsReady(isReady);
  

Serial.println("[INITIALIZING LED MANAGER]");
ledMan = std::make_unique<LedManager>(SDA);
ledMan->setAllLeds(CRGB(10,10,10));

Serial.println("[INITIALIZING BLUETOOTH ENGINE]");
  ble = new BluetoothEngine();    

  ble->setDidReceiveCallback([](std::string data) {              
          rxdData = data;
          didReceiveData = true;                     
  });

  ble->setDidConnectCallback([]() {        
      isConnected = true;
  });

  ble->setDidDisconnectConnectCallback([]() {        
      isConnected = false;
  });


transport->refMachine([](bool success) {    
    machineIsBooted = true;
    Serial.println("[main][refMachineCB] Machine Homed");
});

Serial.println("[main][setup] Done");


ledMan->fadeTo(CRGB(0,0,0), CRGB(100,0,0), 1000);
}


void loop() {  
  transport->heartbeat(); 
  dispenser->heartbeat();
  dispatcher->heartbeat();
  ble->heartbeat();
  ledMan->heartbeat();
  

    if (machineIsBooted == true) {   
              
        Dispatcher::DispatcherState state = dispatcher->getState();
        
          if (state == Dispatcher::DispatcherState::NO_CUP) {                             
                ledMan->fadeTo(ledMan->getCurrentColor(), CRGB(0,0,100), 200);                   
          } else if (state == Dispatcher::DispatcherState::READY) {                
                ledMan->fadeTo(ledMan->getCurrentColor(), CRGB(0,100,0), 350);     
                ble->notifyStatus("Ready!");
          } else if (state == Dispatcher::DispatcherState::AWAITING_REMOVAL) {         
            ledMan->trackTray(transport->getCurrentPosition(), CRGB(0,255,0), CRGB(10,10,10));        
          } else if (dispatcher->isServing() == true) {
            ledMan->trackTray(transport->getCurrentPosition(), CRGB(0,255,255), CRGB(0,0,20));          
          }          
          
      if (state != lastState) { 
        updateCupState();
        lastState = state;
      }
      
    }

  handleSerialRequests();
  handleBleRequests();      
}

void updateCupState() {
  Dispatcher::DispatcherState state = dispatcher->getState();
  if (state == Dispatcher::DispatcherState::NO_CUP) {
          Serial.println("[main][loop] No Cup Detected");
          ble->notifyCupStatus(false);
        } else {
          Serial.println("[main][loop] Cup Detected");
          ble->notifyCupStatus(true);
        }  
}


void handleSerialRequests() {
if (Serial.available() > 0) {  // Check if data is available to read
    auto receivedChar = Serial.read(); // Read the incoming byte
    // Serial.print((int)receivedChar);
    switch (receivedChar) {       
     case '?':
        Serial.println(transport->getCurrentPosition());
        break;
      case ',':
        dispenser->scale_->set_scale(dispenser->scale_->get_scale() - 10);
        Serial.println(String(dispenser->scale_->get_scale()) + " = " + String(dispenser->scale_->get_units()));
        break;
      case '.':
        dispenser->scale_->set_scale(dispenser->scale_->get_scale() + 10);
        Serial.println(String(dispenser->scale_->get_scale()) + " = " + String(dispenser->scale_->get_units()));
        break;        
      case '<': 
       Serial.println("[main][loop] Left");
        transport->moveStepsLeft(20);
        break;
      case  '>': 
      Serial.println("[main][loop] Right");
        transport->moveStepsRight(20);
        break;
      case 'B':
      if (dispatcher->getState() == Dispatcher::DispatcherState::READY) {                        
        // dispatcher->addStep(Dispenser::DispenseType::PUMP, 1, 7, 50.0);  
        // dispatcher->addStep(Dispenser::DispenseType::PUMP, 2, 8, 50.0);  
        // dispatcher->addStep(Dispenser::DispenseType::PUMP, 3, 9, 50.0);  

        dispatcher->addStep(Dispenser::DispenseType::VALVE, 1, 1, 50.0);  //Station 1 // Valve 1 
        dispatcher->addStep(Dispenser::DispenseType::PUMP, 7, 1, 50.0);  //Station 7 // Pump 1
        dispatcher->addStep(Dispenser::DispenseType::PUMP, 7, 2, 50.0);  //Station 7 // Pump 2
        dispatcher->addStep(Dispenser::DispenseType::PUMP, 7, 3, 50.0);  //Station 7 // Pump 3
        dispatcher->addStep(Dispenser::DispenseType::VALVE, 2, 2, 50.0);  //Station 2 // Valve 2 
        dispatcher->start();
      } else {
        Serial.println("[main][loop] Dispatcher not ready.");
      }
        break;
      case 'C':      
       dispatcher->cancel();
       break;   
      case 'T':      
       dispenser->tare();      
        break;            
      case '1':                    
       dispenser->selectValveForTrim(1, Valve::Position::OPEN);
        break;
      case 'q':
        dispenser->selectValveForTrim(1, Valve::Position::CLOSED);
        break;
      case '2':                    
       dispenser->selectValveForTrim(2, Valve::Position::OPEN);
        break;
      case 'w':
        dispenser->selectValveForTrim(2, Valve::Position::CLOSED);
        break;      
      case '3':                    
       dispenser->selectValveForTrim(3, Valve::Position::OPEN);
        break;
      case 'e':
        dispenser->selectValveForTrim(3, Valve::Position::CLOSED);
        break;
      case '4':
        dispenser->selectValveForTrim(4, Valve::Position::OPEN);
        break;
      case 'r':
        dispenser->selectValveForTrim(4, Valve::Position::CLOSED);
        break;
      case '5':
        dispenser->selectValveForTrim(5, Valve::Position::OPEN);
        break;
      case 't':
        dispenser->selectValveForTrim(5, Valve::Position::CLOSED);
        break;
      case '6':
        dispenser->selectValveForTrim(6, Valve::Position::OPEN);
        break;
      case 'y':
        dispenser->selectValveForTrim(6, Valve::Position::CLOSED);
        break;
      case '+':
        dispenser->trimValve(1);
        break;
      case '-':
        dispenser->trimValve(-1);
        break;
      case '@':
        dispenser->resetTrimPositions();
        break;
      case 'P':
        transport->goToStation(7);
        break;
      case 'S':
        Serial.println("[main][loop] Weight: " + String(dispenser->getLatestWeight()));
        break;
      case 'A':
        Serial.println("[main][loop] Absolute Weight: " + String(dispenser->getAbsoluteWeight()));
        break;
      case '#':
        dispenser->beginDispensingPump(1, 50.0);
        break;
      
      default:        
        break;
    }    
        
  }
}

void willBeginDispensing(uint8_t step) {
  Serial.println("[main][willBeginDispensingCallback] Step: " + String(step));
  ble->notifyStateIsProcessing(step);
  ble->notifyStatus("Still Working!");
}

void didFinishDispensing(uint8_t step) {
  ble->notifyStateIsComplete(step);
  Serial.println("[main][didFinishDispensingCallback] Step: " + String(step));  
}

void didUpdateWeight(uint8_t step, float weight) {
  ble->notifyWeightUpdate(step, weight);  
}

void didFinishJob() {
  Serial.println("[main][didFinishJobCallback] Job Complete");  
  ble->notifyStatus("Get your drink!");
}

void isReady() {
  Serial.println("[main][isReadyCallback] Ready");
  ble->notifyStatus("Ready!");
}


void handleBleRequests() {

  std::string rxdData_ = rxdData;
  if (didReceiveData == true) {
    
    didReceiveData = false;
    rxdData = "";

    Serial.println("[Main][handleBleRequests] Received: " + String(rxdData_.c_str()));
    if (parseBleRequestToDispatcher(rxdData_) == true) {
      if (dispatcher->getState() == Dispatcher::DispatcherState::NO_CUP) {
        ble->notifyStatus("No Cup! Please add a cup!");
        ble->notifyCupStatus(false);
        return;
      }
      ble->notifyStatus("Serving your drink!");
      dispatcher->start();
    } else if (rxdData_ == "C!") {
      Serial.println("[Main][handleBleRequests] Cancel Request Received");
      dispatcher->cancel();
    } else if (rxdData_ == "ehlo") {
      Serial.println("[Main][handleBleRequests] Ping Received");
      updateCupState();      
    }  else {
      Serial.println("[Main][handleBleRequests] Unknown Request Received: " + String(rxdData_.c_str()));
    }              
  }
  
}


bool parseBleRequestToDispatcher(const std::string& rxdData) {
    std::vector<Dispatcher::Steps> steps;

    // Find the position of the first ':'
    size_t pos = rxdData.find(':');
    if (pos == std::string::npos) {
        // std::cerr << "Invalid input string format\n";
        return false;
    }

    // Extract the command part (before ':')
    std::string command = rxdData.substr(0, pos);
    if (command != "D") {     
        return false;
    }

    // Extract the remaining part (after ':')
    std::string stepsPart = rxdData.substr(pos + 1);

    // Tokenize the stepsPart by ','
    char stepsCopy[256];
    strncpy(stepsCopy, stepsPart.c_str(), sizeof(stepsCopy));
    stepsCopy[sizeof(stepsCopy) - 1] = '\0'; // Ensure null termination

    char* step = strtok(stepsCopy, ",");
    dispatcher->clearSteps();
    Serial.println("[main][parseBTRequestToDispatcher] --------------------------------->");
    while (step != NULL) {
        
        char* equalPos = strchr(step, '=');
        if (equalPos == NULL) {
            Serial.println("Invalid step format");
            return false;
        }

        // Extract StepID (index) and value
        *equalPos = '\0';
        int addressID = atoi(step);
        double targetWeight = atof(equalPos + 1);

        uint8_t stationID = (uint8_t)addressID;
        uint8_t pourDeviceID = stationID;

        
        Dispenser::DispenseType stationType = Dispenser::DispenseType::VALVE;
        if (stationID >= 7) {
            stationType = Dispenser::DispenseType::PUMP;
            stationID = 7;
            pourDeviceID = addressID - 6;
        }

        dispatcher->addStep(stationType, stationID, pourDeviceID, targetWeight);

        Serial.println("[main][parseBTRequestToDispatcher] Step Added: " + String(addressID) + " = " + String(targetWeight)); 
        step = strtok(NULL, ",");
    }

    Serial.println("[main][parseBTRequestToDispatcher] ---------------------------------<");
    return true;
}
