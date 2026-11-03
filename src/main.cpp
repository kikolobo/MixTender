#include <iostream>
#include <vector>
#include <cstring>
#include <atomic>
#include <deque>
#include <mutex>
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

// Job limits enforced on every BLE "D:" request.
const uint8_t NUM_VALVE_STATIONS = 6;   // ids 1..6 -> valves 1..6 at stations 1..6
const uint8_t NUM_PUMPS = 3;            // ids 7..9 -> pumps 1..3, all at station 7
const uint8_t PUMP_STATION = 7;
const float MAX_JOB_GRAMS = 450.0;      // Glass capacity. Reject jobs above this.
const float MAX_START_GRAMS = 400.0;    // Heaviest *empty* cup accepted. Above this the cup
                                        // probably still holds a drink: refuse to pour.
const size_t MAX_PENDING_BLE_REQUESTS = 8;

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


// BLE writes arrive on the BLE task; loop() drains them. The mutex guards the
// queue (std::string allocates, so it can't be shared without a lock).
std::mutex bleRequestsMutex;
std::deque<std::string> bleRequests;
std::atomic<bool> isConnected(false);
Dispatcher::DispatcherState lastState = Dispatcher::DispatcherState::UNKNOWN;

void handleBleRequests();
void handleSerialRequests();

struct ParsedStep {
  Dispenser::DispenseType type;
  uint8_t stationID;
  uint8_t pourDeviceID; // 1-based, as Dispatcher::addStep expects
  float targetWeight;
};
// Fills `steps` with every pair it finds (stepCount = number of pairs, even if
// some are invalid, so a rejection can fail each of them). Returns false and
// sets `error` if any pair is invalid or the total exceeds MAX_JOB_GRAMS.
bool parseJob(const std::string& request, std::vector<ParsedStep>& steps, size_t& stepCount, String& error);
void rejectJob(size_t stepCount, const std::string& reason);
std::string statusTextForState();
void handleTrimRequest(const std::string& request);
void sendTrimReply(const std::string& message);

//Callbacks from dispatcher (prototypes)
void willBeginDispensing(uint8_t step);
void didFinishDispensing(uint8_t step);
void didFailDispensing(uint8_t step);
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
  dispenser->loadValveTrims(); // Saved calibration overrides the 50/147 defaults per valve.

  //Register Pumps
  dispenser->registerPump(std::make_shared<Pump>(CH1_PIN));
  dispenser->registerPump(std::make_shared<Pump>(CH2_PIN));
  dispenser->registerPump(std::make_shared<Pump>(CH3_PIN));

  Serial.println("[INITIALIZING DISPATCHER]");
  dispatcher->setWillBeginDispensingCallback(willBeginDispensing);
  dispatcher->setDidFinishDispensingCallback(didFinishDispensing);
  dispatcher->setDidFailDispensingCallback(didFailDispensing);
  dispatcher->setDidUpdateWeight(didUpdateWeight);
  dispatcher->setDidFinishJob(didFinishJob);
  dispatcher->setIsReady(isReady);
  

Serial.println("[INITIALIZING LED MANAGER]");
ledMan = std::make_unique<LedManager>(SDA);
ledMan->setAllLeds(CRGB(10,10,10));

Serial.println("[INITIALIZING BLUETOOTH ENGINE]");
  ble = new BluetoothEngine();    

  ble->setDidReceiveCallback([](const std::string& data) {
      std::lock_guard<std::mutex> lock(bleRequestsMutex);
      if (bleRequests.size() >= MAX_PENDING_BLE_REQUESTS) {
        bleRequests.pop_front(); // Drop the oldest rather than grow without bound.
      }
      bleRequests.push_back(data);
  });

  ble->setDidConnectCallback([]() {        
      isConnected = true;
  });

  ble->setDidDisconnectConnectCallback([]() {        
      isConnected = false;
      // Don't leave a valve open if the app disconnects mid-calibration.
      // Queued so it runs on loop(), not the BLE task (ignored while pouring).
      std::lock_guard<std::mutex> lock(bleRequestsMutex);
      bleRequests.push_back("TX");
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

    // Valve calibration keys move servos: never while a job is running.
    if (strchr("123456qwerty+-@!", receivedChar) != NULL && dispatcher->isServing()) {
      Serial.println("[Trim] Ignored: a drink is being poured.");
      return;
    }
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
      case '!':
        dispenser->saveSelectedValveTrim();
        break;
      case 'V':
        dispenser->printValveTrims();
        break;
      case 'h':
        Serial.println("--- MixTender serial commands ---");
        Serial.println(" Valve calibration (blocked while pouring):");
        Serial.println("  1-6      select valve 1-6, move OPEN");
        Serial.println("  q w e r t y  select valve 1-6, move CLOSED");
        Serial.println("  + / -    nudge selected valve's current position by 1 deg");
        Serial.println("  @        reset selected valve to defaults (50/147), not saved");
        Serial.println("  !        save selected valve's open+closed trims to flash");
        Serial.println("  V        list all valves: angles, trims, saved/defaults");
        Serial.println(" Other:");
        Serial.println("  ? position   < > jog tray   P pump station   B test job   C cancel");
        Serial.println("  T tare   S weight   A absolute weight   , . scale factor");
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

void didFailDispensing(uint8_t step) {
  ble->notifyStateIsFailed(step);
  Serial.println("[main][didFailDispensingCallback] Step: " + String(step));
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

  std::string rxdData_;
  {
    std::lock_guard<std::mutex> lock(bleRequestsMutex);
    if (bleRequests.empty()) {
      return;
    }
    rxdData_ = std::move(bleRequests.front());
    bleRequests.pop_front();
  }
  {

    Serial.println("[Main][handleBleRequests] Received: " + String(rxdData_.c_str()));

    if (rxdData_.rfind("D:", 0) == 0) {
      std::vector<ParsedStep> steps;
      size_t stepCount = 0;
      String error;
      bool valid = parseJob(rxdData_, steps, stepCount, error);

      // Never touch the Dispatcher's steps while a job is running or the tray
      // is still parking after a cancel: it is iterating over them.
      if (dispatcher->isServing() || !transport->isParked()) {
        rejectJob(stepCount, "Busy, try again");
        return;
      }
      if (dispatcher->getState() == Dispatcher::DispatcherState::NO_CUP) {
        rejectJob(stepCount, "No Cup! Please add a cup!");
        ble->notifyCupStatus(false);
        return;
      }
      if (dispatcher->getState() != Dispatcher::DispatcherState::READY) {
        // e.g. AWAITING_REMOVAL: the finished drink is still on the tray.
        rejectJob(stepCount, "Remove your drink first");
        return;
      }
      float cupWeight = dispenser->getAbsoluteWeight();
      if (cupWeight > MAX_START_GRAMS) {
        Serial.println("[Main][handleBleRequests] Cup too heavy to be empty: " + String(cupWeight) + "g");
        rejectJob(stepCount, "Cup is not empty");
        return;
      }
      if (!valid) {
        Serial.println("[Main][handleBleRequests] Rejected job: " + error);
        rejectJob(stepCount, "Invalid drink request");
        return;
      }

      dispatcher->clearSteps();
      for (const auto& step : steps) {
        dispatcher->addStep(step.type, step.stationID, step.pourDeviceID, step.targetWeight);
      }
      if (!dispatcher->start()) {
        dispatcher->clearSteps();
        rejectJob(stepCount, "Not ready, try again");
        return;
      }
      ble->notifyStatus("Serving your drink!");

    } else if (rxdData_ == "C!") {
      Serial.println("[Main][handleBleRequests] Cancel Request Received");
      if (dispatcher->cancel()) {
        ble->notifyStatus("Cancelled");
      }
    } else if (rxdData_ == "ehlo") {
      Serial.println("[Main][handleBleRequests] Ping Received");
      updateCupState();
      ble->notifyStatus(statusTextForState());
    } else if (!rxdData_.empty() && rxdData_[0] == 'T') {
      handleTrimRequest(rxdData_);
    }  else {
      Serial.println("[Main][handleBleRequests] Unknown Request Received: " + String(rxdData_.c_str()));
    }
  }

}

void sendTrimReply(const std::string& message) {
  ble->sendData(message, true); // Always reply, even if identical to the last message.
}

// Valve calibration over BLE. See NOTES_FROM_FIRMWARE.md in the app repo.
//   T?            list all valves -> one T<id>=... line per valve
//   TO<id>        move valve to OPEN          TC<id>        move to CLOSED
//   TO<id>=<deg>  set OPEN angle and move     TC<id>=<deg>  set CLOSED angle and move
//   TW<id>        save valve's trims to flash TR<id>        reset to defaults (not saved)
//   TX            close all valves (end of calibration) -> TX=ok
// Replies T<id>=<open>,<closed>,<defOpen>,<defClosed>,<saved> or TE=<error>.
void handleTrimRequest(const std::string& request) {
  if (request == "T?") {
    for (uint8_t id = 1; id <= dispenser->getValveCount(); id++) {
      sendTrimReply(dispenser->valveTrimStatus(id));
      delay(5); // Don't flood the BLE stack with back-to-back notifications.
    }
    return;
  }

  if (dispatcher->isServing()) {
    sendTrimReply("TE=busy"); // Never move valves during a pour.
    return;
  }

  if (request == "TX") {
    dispenser->setAllValves(Valve::Position::CLOSED);
    sendTrimReply("TX=ok");
    return;
  }

  if (request.size() < 3) {
    sendTrimReply("TE=bad");
    return;
  }
  char command = request[1];
  char* end;
  long valveId = strtol(request.c_str() + 2, &end, 10);
  if (end == request.c_str() + 2) {
    sendTrimReply("TE=bad");
    return;
  }
  if (valveId < 1 || valveId > dispenser->getValveCount()) {
    sendTrimReply("TE=valve");
    return;
  }

  bool hasAngle = (*end == '=');
  long angle = 0;
  if (hasAngle) {
    char* angleStart = end + 1;
    angle = strtol(angleStart, &end, 10);
    if (end == angleStart || *end != '\0') {
      sendTrimReply("TE=bad");
      return;
    }
    if (angle < 0 || angle > 180) {
      sendTrimReply("TE=angle");
      return;
    }
  } else if (*end != '\0') {
    sendTrimReply("TE=bad");
    return;
  }

  bool ok = true;
  switch (command) {
    case 'O':
    case 'C': {
      Valve::Position position = (command == 'O') ? Valve::Position::OPEN : Valve::Position::CLOSED;
      if (hasAngle) {
        ok = dispenser->setValveAngle(valveId, position, angle);
      } else {
        dispenser->selectValveForTrim(valveId, position);
      }
      break;
    }
    case 'W':
      if (hasAngle) { sendTrimReply("TE=bad"); return; }
      if (!dispenser->saveValveTrim(valveId)) { sendTrimReply("TE=flash"); return; }
      break;
    case 'R':
      if (hasAngle) { sendTrimReply("TE=bad"); return; }
      ok = dispenser->resetValveTrim(valveId);
      break;
    default:
      sendTrimReply("TE=bad");
      return;
  }

  sendTrimReply(ok ? dispenser->valveTrimStatus(valveId) : "TE=bad");
}

std::string statusTextForState() {
  if (!machineIsBooted) {
    return "Starting up...";
  }
  switch (dispatcher->getState()) {
    case Dispatcher::DispatcherState::NO_CUP:           return "Please add a cup";
    case Dispatcher::DispatcherState::READY:            return "Ready!";
    case Dispatcher::DispatcherState::AWAITING_REMOVAL: return "Get your drink!";
    case Dispatcher::DispatcherState::JOB_COMPLETE:     return "Parking...";
    default:                                            return "Still Working!";
  }
}

// The app has no job-level reject message. Failing every step lets its pour
// screen finish instead of waiting forever.
void rejectJob(size_t stepCount, const std::string& reason) {
  for (size_t i = 0; i < stepCount; i++) {
    ble->notifyStateIsFailed(i);
    delay(5); // Don't flood the BLE stack with back-to-back notifications.
  }
  ble->notifyStatus(reason);
}


bool parseJob(const std::string& request, std::vector<ParsedStep>& steps, size_t& stepCount, String& error) {
    steps.clear();
    stepCount = 0;
    error = "";

    // Copy everything after "D:" so strtok can split it.
    char stepsCopy[256];
    strncpy(stepsCopy, request.c_str() + 2, sizeof(stepsCopy));
    stepsCopy[sizeof(stepsCopy) - 1] = '\0';

    float totalWeight = 0.0;
    Serial.println("[main][parseJob] --------------------------------->");
    for (char* pair = strtok(stepsCopy, ","); pair != NULL; pair = strtok(NULL, ",")) {
        stepCount++;
        if (error.length() > 0) {
            continue; // Already invalid: keep counting so every step can be failed.
        }

        char* equalPos = strchr(pair, '=');
        if (equalPos == NULL) {
            error = "step " + String(stepCount - 1) + ": missing '='";
            continue;
        }
        *equalPos = '\0';
        char* end;
        long addressID = strtol(pair, &end, 10);
        if (end == pair || *end != '\0' || addressID < 1 || addressID > NUM_VALVE_STATIONS + NUM_PUMPS) {
            error = "step " + String(stepCount - 1) + ": unknown station '" + String(pair) + "'";
            continue;
        }
        float targetWeight = strtof(equalPos + 1, &end);
        if (end == equalPos + 1 || *end != '\0' || !isfinite(targetWeight) || targetWeight < 0.0) {
            error = "step " + String(stepCount - 1) + ": bad amount '" + String(equalPos + 1) + "'";
            continue;
        }

        ParsedStep step;
        step.targetWeight = targetWeight;
        if (addressID <= NUM_VALVE_STATIONS) {
            step.type = Dispenser::DispenseType::VALVE;
            step.stationID = (uint8_t)addressID;
            step.pourDeviceID = (uint8_t)addressID;
        } else {
            step.type = Dispenser::DispenseType::PUMP;
            step.stationID = PUMP_STATION;
            step.pourDeviceID = (uint8_t)(addressID - NUM_VALVE_STATIONS);
        }
        steps.push_back(step);
        totalWeight += targetWeight;
        Serial.println("[main][parseJob] Step: " + String(addressID) + " = " + String(targetWeight));
    }
    Serial.println("[main][parseJob] ---------------------------------<");

    if (error.length() == 0 && stepCount == 0) {
        error = "no steps";
    }
    if (error.length() == 0 && totalWeight > MAX_JOB_GRAMS) {
        error = "total " + String(totalWeight) + "g exceeds max " + String(MAX_JOB_GRAMS) + "g";
    }
    return error.length() == 0;
}
