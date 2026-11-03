#include <iostream>
#include <vector>
#include <cstring>
#include <atomic>
#include <deque>
#include <mutex>
#include <Arduino.h>
#include <Preferences.h>

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
const float DEFAULT_EMPTY_TRAY_GRAMS = 121.38; // Used until an empty-tray weight is saved (K / Z)

// Job limits enforced on every BLE "D:" request.
const uint8_t NUM_VALVE_STATIONS = 6;   // ids 1..6 -> valves 1..6 at stations 1..6
const uint8_t NUM_PUMPS = 3;            // ids 7..9 -> pumps 1..3, all at station 7
const uint8_t PUMP_STATION = 7;
const float MAX_JOB_GRAMS = 450.0;      // Glass capacity. Reject jobs above this.
const float DEFAULT_MAX_CUP_GRAMS = 450.0; // Heaviest *empty* cup accepted (default). Above this the
                                          // cup probably still holds a drink: refuse to pour.
// Cup weight window, both over the empty tray and saved in flash:
//   below min        -> no cup ("Please add a cup")
//   min .. max       -> empty cup, ready
//   above max        -> "Cup is not empty"
// Settable range for both: CUP_LIMIT_LOW..CUP_LIMIT_HIGH, and min must stay below max.
const float CUP_LIMIT_LOW = 5.0;
const float CUP_LIMIT_HIGH = 2000.0;
const float MIN_CUP_GAP_GRAMS = 20.0;      // min must be at least this much below max
float maxCupGrams = DEFAULT_MAX_CUP_GRAMS; // In use; a saved value (KM= / M) overrides the default
const float DEFAULT_MIN_CUP_GRAMS = Dispatcher::DEFAULT_MIN_CUP_GRAMS; // KN= / N
void loadCupLimits();               // Apply saved min/max (call once the Dispatcher exists).
// Set and save. False if out of range, too close to the other limit, or the save failed.
bool setMinCupGrams(float grams);
bool setMaxCupGrams(float grams);
// Back to the default, saved value erased. False if the default would cross the other limit.
bool resetMinCupGrams();
bool resetMaxCupGrams();
bool isCupSettingSaved(const char* key);
bool isSaveableCupLimit(bool isMin, float grams); // In range and not crossing the other limit
static const char* CUP_NVS_NAMESPACE = "scale";
static const char* MIN_CUP_KEY = "mincup";
static const char* MAX_CUP_KEY = "maxcup";
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
void onHomed(bool success);
void didAbortJob(Dispatcher::AbortReason reason);
bool homingFailed = false;

// Robot status pushed to the app (see updateRobotStatus).
bool okToServe();                 // What $1 means to the app: "SERVE may be shown"
const char* idleStatusText();     // $0 text for idle/busy states, nullptr when a job event text should stand
void updateRobotStatus();
int8_t lastOkToServe = -1;        // -1 = not sent yet
std::string lastIdleText;
bool cupNotEmpty = false;         // Cup on the scale is heavier than maxCupGrams (with hysteresis)
std::atomic<bool> bleDisconnectedFlag(false);

// Calibration tray session (TM over BLE, G over serial). The tray is moved under a
// valve so a calibration "open" pours into the cup instead of onto the machine.
bool calibrationTrayActive = false;   // Tray was moved for calibration: park it on TX/X.
int8_t calibrationTrayTarget = -1;    // Station we're moving to (-1 = none pending).
bool calibrationTrayNotifyBle = false; // Send TM<id>=at on arrival.
const char* checkCalibrationTrayMove(); // nullptr if allowed, else TE code
void moveTrayForCalibration(uint8_t station, bool fromBle);
const char* checkValveOpenAllowed(uint8_t valveId); // nullptr if allowed, else TE code
void endCalibrationSession();
void checkCalibrationTrayArrival();
void sendTrimReply(const std::string& message);

// Empty-tray weight (K over BLE, K/Z over serial).
void handleScaleRequest(const std::string& request);
const char* checkScaleChangeAllowed(bool needsReading); // nullptr if allowed, else KE code
std::string scaleStatusReply();

//Callbacks from dispatcher (prototypes)
void willBeginDispensing(uint8_t step);
void didFinishDispensing(uint8_t step);
void didFailDispensing(uint8_t step);
void didUpdateWeight(uint8_t step, float weight);
void didFinishJob();
void isReady();

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
  dispenser = std::make_shared<Dispenser>(LC_DAT, LC_SCK, calibration_factor, DEFAULT_EMPTY_TRAY_GRAMS);
  dispenser->loadEmptyWeight(); // Saved empty-tray weight (K/Z) overrides the default.
  dispatcher = std::make_unique<Dispatcher>(dispenser, transport);
  loadCupLimits();              // Saved min/max cup weights (KN=/KM=, N/M) override the defaults.

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
  dispatcher->setDidAbortJobCallback(didAbortJob);
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
      bleDisconnectedFlag = true; // loop() aborts a running job (the app won't resume it).
      // Don't leave a valve open if the app disconnects mid-calibration.
      // Queued so it runs on loop(), not the BLE task (ignored while pouring).
      std::lock_guard<std::mutex> lock(bleRequestsMutex);
      bleRequests.push_back("TX");
  });


transport->refMachine(onHomed);

Serial.println("[main][setup] Done");


ledMan->fadeTo(CRGB(0,0,0), CRGB(100,0,0), 1000);

// Reset the board if loop() ever stops running for ~5 s (a hang), instead of
// freezing. Safe: on boot every valve closes and every pump starts off.
enableLoopWDT();
}

void onHomed(bool success) {
  if (success) {
    homingFailed = false;
    machineIsBooted = true;
    Serial.println("[main][refMachineCB] Machine Homed");
  } else {
    homingFailed = true;
    machineIsBooted = false;
    Serial.println("[main][refMachineCB] HOMING FAILED. Check the home switch, then press H to retry.");
    if (ble != nullptr) {
      ble->notifyStatus("Homing failed. Restart robot");
    }
  }
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
                if (okToServe()) {
                  ledMan->fadeTo(ledMan->getCurrentColor(), CRGB(0,100,0), 350);   // Green: clear to serve
                } else {
                  // Yellow: cup on, but not clear to go (cup not empty, calibrating, parking...).
                  ledMan->fadeTo(ledMan->getCurrentColor(), CRGB(100,70,0), 350);
                }
          } else if (state == Dispatcher::DispatcherState::AWAITING_REMOVAL) {         
            ledMan->trackTray(transport->getCurrentPosition(), CRGB(0,255,0), CRGB(10,10,10));        
          } else if (dispatcher->isServing() == true) {
            ledMan->trackTray(transport->getCurrentPosition(), CRGB(0,255,255), CRGB(0,0,20));          
          }          
          
    }

  if (bleDisconnectedFlag.exchange(false) && dispatcher->abort(Dispatcher::AbortReason::DISCONNECTED)) {
    Serial.println("[main][loop] App disconnected mid-job. Job stopped.");
  }
  updateRobotStatus();
  checkCalibrationTrayArrival();
  handleSerialRequests();
  handleBleRequests();      
}

bool okToServe() {
  return !homingFailed && machineIsBooted && dispenser->isScaleResponding()
      && dispatcher->getState() == Dispatcher::DispatcherState::READY
      && transport->isParked() && !cupNotEmpty;
}

const char* idleStatusText() {
  if (homingFailed)                                               return "Homing failed. Restart robot";
  if (!dispenser->isScaleResponding())                            return "Scale error";
  if (transport->getState() == Transport::MachineState::HOMING)   return "Homing...";
  if (!machineIsBooted)                                           return "Starting up...";
  if (dispatcher->isServing())                                    return nullptr; // Job texts (Still Working!, ...)
  switch (dispatcher->getState()) {
    case Dispatcher::DispatcherState::AWAITING_REMOVAL: return nullptr; // Keep "Get your drink!" / the abort reason
    case Dispatcher::DispatcherState::JOB_COMPLETE:     return nullptr; // Keep "Cancelled" while parking
    default: break;
  }
  if (calibrationTrayActive)                                      return "Calibrating...";
  if (!transport->isParked())                                     return "Parking...";
  if (dispatcher->getState() == Dispatcher::DispatcherState::NO_CUP) return "Please add a cup";
  if (cupNotEmpty)                                                return "Cup is not empty";
  return "Ready!";
}

// Push $1 (OK to serve) and the idle $0 text to the app whenever they change, so
// SERVE is hidden before the user taps it instead of after a rejected job.
void updateRobotStatus() {
  Dispatcher::DispatcherState state = dispatcher->getState();
  if (state == Dispatcher::DispatcherState::READY) {
    float w = dispenser->getAbsoluteWeight();
    if (w > maxCupGrams) {
      cupNotEmpty = true;
    } else if (w < maxCupGrams - 10.0) {
      cupNotEmpty = false; // 10 g hysteresis so a cup near the limit doesn't flicker
    }
  } else if (state == Dispatcher::DispatcherState::NO_CUP) {
    cupNotEmpty = false;
  }

  bool ok = okToServe();
  if ((int8_t)ok != lastOkToServe) {
    Serial.println(String("[main][status] OK to serve: ") + (ok ? "yes" : "no"));
    ble->notifyCupStatus(ok);
    lastOkToServe = ok;
  }

  const char* text = idleStatusText();
  if (text == nullptr) {
    lastIdleText = ""; // A job/event text is showing; re-send the idle text when we're back.
  } else if (lastIdleText != text) {
    ble->notifyStatus(text);
    lastIdleText = text;
  }
}


void handleSerialRequests() {
if (Serial.available() > 0) {  // Check if data is available to read
    auto receivedChar = Serial.read(); // Read the incoming byte

    // "K" (empty-tray weight), "N" (min cup weight) or "M" (max cup weight), then a number and Enter.
    // "R" + Enter = default, Enter alone = cancel. Captured before anything else so
    // the digits don't open valves.
    static char pendingEntry = 0; // 'K', 'N', 'M', or 0
    static String entryLine;
    if (pendingEntry != 0) {
      if (receivedChar != '\r' && receivedChar != '\n') {
        if (entryLine.length() < 16) {
          entryLine += (char)receivedChar;
        }
        return;
      }
      char entry = pendingEntry;
      pendingEntry = 0;
      String line = entryLine;
      entryLine = "";
      line.trim();
      if (line.length() == 0) {
        Serial.println("[Scale] Cancelled.");
        return;
      }
      const char* error = checkScaleChangeAllowed(false);
      if (error != nullptr) {
        Serial.println("[Scale] Can't change it now (" + String(error) + ").");
        return;
      }
      bool reset = (line == "R" || line == "r");
      char* end;
      float grams = reset ? 0.0f : strtof(line.c_str(), &end);
      bool parsed = reset || (end != line.c_str() && *end == '\0');
      bool ok;
      if (entry == 'K') {
        ok = parsed && (reset ? dispenser->resetEmptyWeight() : dispenser->setEmptyWeight(grams));
      } else if (entry == 'N') {
        ok = parsed && (reset ? resetMinCupGrams() : setMinCupGrams(grams));
      } else {
        ok = parsed && (reset ? resetMaxCupGrams() : setMaxCupGrams(grams));
      }
      if (!ok) {
        Serial.println("[Scale] Not set: '" + line + "' (empty tray: +/-" + String(Dispenser::MAX_EMPTY_WEIGHT_GRAMS, 0)
                       + "g; cup min/max: " + String(CUP_LIMIT_LOW, 0) + "-" + String(CUP_LIMIT_HIGH, 0)
                       + "g with min at least " + String(MIN_CUP_GAP_GRAMS, 0) + "g below max).");
        return;
      }
      Serial.println("[Scale] Absolute weight now " + String(dispenser->getAbsoluteWeight(), 1)
                     + "g, cup window " + String(dispatcher->getMinCupGrams(), 1) + "-" + String(maxCupGrams, 1) + "g.");
      return;
    }
    if (receivedChar == 'K') {
      pendingEntry = 'K';
      entryLine = "";
      Serial.println("[Scale] Empty tray weight is " + String(dispenser->getEmptyWeight(), 2)
                     + "g. Type new grams + Enter (R + Enter = default " + String(dispenser->getDefaultEmptyWeight(), 2)
                     + "g, Enter alone = cancel).");
      return;
    }
    if (receivedChar == 'N') {
      pendingEntry = 'N';
      entryLine = "";
      Serial.println("[Scale] Min cup weight (lightest empty cup) is " + String(dispatcher->getMinCupGrams(), 1)
                     + "g. Type new grams + Enter (R + Enter = default " + String(DEFAULT_MIN_CUP_GRAMS, 1)
                     + "g, Enter alone = cancel).");
      return;
    }
    if (receivedChar == 'M') {
      pendingEntry = 'M';
      entryLine = "";
      Serial.println("[Scale] Max empty-cup weight is " + String(maxCupGrams, 1)
                     + "g. Type new grams + Enter (R + Enter = default " + String(DEFAULT_MAX_CUP_GRAMS, 1)
                     + "g, Enter alone = cancel).");
      return;
    }

    // "G" then a digit 0-6: move the tray under that valve for calibration (0 = park).
    static bool awaitingStationDigit = false;
    if (awaitingStationDigit) {
      awaitingStationDigit = false;
      if (receivedChar >= '7' && receivedChar <= '9') {
        Serial.println("[Calibration] Stations 7-9 are pumps (on/off): only valves 1-6 can be calibrated.");
        return;
      }
      if (receivedChar < '0' || receivedChar > '6') {
        Serial.println("[Calibration] G needs a station 0-6. Cancelled.");
        return;
      }
      const char* error = checkCalibrationTrayMove();
      if (error != nullptr) {
        Serial.println("[Calibration] Can't move tray now (" + String(error) + ").");
        return;
      }
      moveTrayForCalibration(receivedChar - '0', false);
      return;
    }
    if (receivedChar == 'G') {
      awaitingStationDigit = true;
      Serial.println("[Calibration] Station? (0-6)");
      return;
    }

    // Opening (or nudging an open valve) pours: only with the tray under that valve.
    {
      uint8_t openId = 0;
      if (receivedChar >= '1' && receivedChar <= '6') {
        openId = receivedChar - '0';
      } else if ((receivedChar == '+' || receivedChar == '-') && dispenser->getSelectedTrimValveId() != 0
                 && dispenser->getValvePosition(dispenser->getSelectedTrimValveId()) == Valve::Position::OPEN) {
        openId = dispenser->getSelectedTrimValveId();
      }
      if (openId != 0 && !dispatcher->isServing()) {
        const char* error = checkValveOpenAllowed(openId);
        if (error != nullptr) {
          Serial.println("[Calibration] Valve " + String(openId) + " not opened: tray "
                         + (strcmp(error, "moving") == 0 ? String("still moving.") : "not under it. Press G" + String(openId) + " first."));
          return;
        }
      }
    }

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
       dispenser->tare(); // Non-blocking, refused during a pour
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
      case 'H':
        if (dispatcher->isServing()) {
          Serial.println("[main] Ignored: a drink is being poured.");
        } else {
          endCalibrationSession();
          machineIsBooted = false;
          transport->refMachine(onHomed);
        }
        break;
      case 'X':
        if (dispatcher->isServing()) {
          Serial.println("[Calibration] Ignored: a drink is being poured.");
        } else {
          endCalibrationSession(); // Closes all valves, parks if the tray was moved.
          Serial.println("[Calibration] All valves closed.");
        }
        break;
      case 'h':
        Serial.println("--- MixTender serial commands ---");
        Serial.println(" Valve calibration (blocked while pouring):");
        Serial.println("  G0-G6    move tray under valve 1-6 (G0 = park). Needed before opening");
        Serial.println("  1-6      select valve 1-6, move OPEN (tray must be under it)");
        Serial.println("  q w e r t y  select valve 1-6, move CLOSED");
        Serial.println("  + / -    nudge selected valve's current position by 1 deg");
        Serial.println("  @        reset selected valve to defaults (50/147), not saved");
        Serial.println("  !        save selected valve's open+closed trims to flash");
        Serial.println("  V        list all valves: angles, trims, saved/defaults");
        Serial.println("  X        end calibration: close all valves, park tray");
        Serial.println(" Other:");
        Serial.println("  ? position   < > jog tray   P pump station   B test job   C cancel");
        Serial.println("  T tare   S weight   A absolute weight   , . scale factor   H re-home");
        Serial.println(" Empty tray weight (saved; used for cup detection):");
        Serial.println("  Z        zero: take the current reading as the empty tray (remove the cup first)");
        Serial.println("  K        type grams + Enter to set it, R + Enter = default");
        Serial.println("  N        min cup weight (lightest empty cup): grams + Enter, R + Enter = default");
        Serial.println("  M        max empty-cup weight: type grams + Enter, R + Enter = default");
        break;
      case 'P':
        transport->goToStation(7);
        break;
      case 'S':
        Serial.println("[main][loop] Weight: " + String(dispenser->getLatestWeight()) + "g (scale " + String(dispenser->getSampleRate(), 1) + " SPS)");
        break;
      case 'A':
        Serial.println("[main][loop] Absolute Weight: " + String(dispenser->getAbsoluteWeight())
                       + "g (gross " + String(dispenser->getGrossWeight()) + "g - empty tray "
                       + String(dispenser->getEmptyWeight()) + "g), cup window " + String(dispatcher->getMinCupGrams())
                       + "-" + String(maxCupGrams) + "g");
        break;
      case 'Z': {
        const char* error = checkScaleChangeAllowed(true);
        if (error != nullptr) {
          Serial.println("[Scale] Can't zero now (" + String(error) + "). Tray must be parked, empty, and no job running.");
        } else if (dispenser->setEmptyWeight(dispenser->getGrossWeight())) {
          Serial.println("[Scale] Zeroed: the current reading is now the empty tray.");
        }
        break;
      }
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

void didAbortJob(Dispatcher::AbortReason reason) {
  const char* text = "Stopped";
  switch (reason) {
    case Dispatcher::AbortReason::CUP_REMOVED:  text = "Cup removed! Stopped"; break;
    case Dispatcher::AbortReason::NO_FLOW:      text = "Bottle empty? Stopped"; break;
    case Dispatcher::AbortReason::POUR_TIMEOUT: text = "Pour too slow. Stopped"; break;
    case Dispatcher::AbortReason::SCALE:        text = "Scale error. Stopped"; break;
    case Dispatcher::AbortReason::DISCONNECTED: text = "Disconnected. Stopped"; break;
  }
  Serial.println("[main][didAbortJobCallback] Job aborted: " + String(text));
  ble->notifyStatus(text);
}

void didUpdateWeight(uint8_t step, float weight) {
  ble->notifyWeightUpdate(step, weight);  
}

void didFinishJob() {
  Serial.println("[main][didFinishJobCallback] Job Complete");  
  ble->notifyStatus("Get your drink!");
}

void isReady() {
  // Cup removed after a drink. updateRobotStatus() sends the right text/state.
  Serial.println("[main][isReadyCallback] Drink removed");
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
      if (homingFailed) {
        rejectJob(stepCount, "Homing failed. Restart robot");
        return;
      }
      if (!dispenser->isScaleResponding()) {
        rejectJob(stepCount, "Scale error");
        return;
      }
      if (dispatcher->isServing() || !transport->isParked()) {
        if (calibrationTrayActive) {
          endCalibrationSession(); // App forgot TX: close valves and park so the next try works.
        }
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
      if (cupWeight > maxCupGrams) {
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
      ble->notifyCupStatus(okToServe());
      ble->notifyStatus(statusTextForState());
    } else if (!rxdData_.empty() && rxdData_[0] == 'T') {
      handleTrimRequest(rxdData_);
    } else if (!rxdData_.empty() && rxdData_[0] == 'K') {
      handleScaleRequest(rxdData_);
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
      sendTrimReply(dispenser->valveTrimStatus(id)); // Queued; BluetoothEngine paces sends.
    }
    return;
  }

  if (dispatcher->isServing()) {
    sendTrimReply("TE=busy"); // Never move valves during a pour.
    return;
  }

  if (request == "TX") {
    endCalibrationSession();
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
  if (valveId > NUM_VALVE_STATIONS && valveId <= NUM_VALVE_STATIONS + NUM_PUMPS) {
    sendTrimReply("TE=pump"); // Stations 7-9 are on/off pumps: nothing to calibrate.
    return;
  }
  long minId = (command == 'M') ? 0 : 1; // TM0 = park
  if (valveId < minId || valveId > dispenser->getValveCount()) {
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
    case 'M': {
      if (hasAngle) { sendTrimReply("TE=bad"); return; }
      const char* error = checkCalibrationTrayMove();
      if (error != nullptr) { sendTrimReply(std::string("TE=") + error); return; }
      moveTrayForCalibration(valveId, true);
      sendTrimReply("TM" + std::to_string(valveId) + "=moving");
      return; // "TM<id>=at" follows on arrival (checkCalibrationTrayArrival).
    }
    case 'O':
    case 'C': {
      Valve::Position position = (command == 'O') ? Valve::Position::OPEN : Valve::Position::CLOSED;
      if (position == Valve::Position::OPEN) {
        const char* error = checkValveOpenAllowed(valveId);
        if (error != nullptr) { sendTrimReply(std::string("TE=") + error); return; }
      }
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

// Empty-tray weight over BLE.
//   K?          status                  -> K=<absolute>,<emptyTray>,<default>,<saved>
//   KZ          zero: current reading becomes the empty tray, saved
//   KT=<grams>  set the empty-tray weight, saved
//   KR          back to the default, saved value erased
//   KM=<grams>  set the max empty-cup weight, saved
//   KMR         max cup weight back to the default, saved value erased
//   KN=<grams>  set the min cup weight (lightest empty cup), saved
//   KNR         min cup weight back to the default, saved value erased
// Reply: K=<absolute>,<emptyTray>,<defaultTray>,<traySaved>,<maxCup>,<defaultMaxCup>,<maxCupSaved>,
//          <minCup>,<defaultMinCup>,<minCupSaved>
// Errors: KE=busy|scale|range|bad|flash
void handleScaleRequest(const std::string& request) {
  if (request == "K?") {
    sendTrimReply(scaleStatusReply());
    return;
  }

  if (request == "KZ") {
    const char* error = checkScaleChangeAllowed(true);
    if (error != nullptr) { sendTrimReply(std::string("KE=") + error); return; }
    if (!dispenser->setEmptyWeight(dispenser->getGrossWeight())) { sendTrimReply("KE=flash"); return; }
  } else if (request == "KR") {
    const char* error = checkScaleChangeAllowed(false);
    if (error != nullptr) { sendTrimReply(std::string("KE=") + error); return; }
    if (!dispenser->resetEmptyWeight()) { sendTrimReply("KE=flash"); return; }
  } else if (request == "KMR" || request == "KNR") {
    const char* error = checkScaleChangeAllowed(false);
    if (error != nullptr) { sendTrimReply(std::string("KE=") + error); return; }
    bool isMin = (request == "KNR");
    if (!(isMin ? resetMinCupGrams() : resetMaxCupGrams())) {
      // Fails only if the default would cross the other limit (or flash failed).
      sendTrimReply(isSaveableCupLimit(isMin, isMin ? DEFAULT_MIN_CUP_GRAMS : DEFAULT_MAX_CUP_GRAMS) ? "KE=flash" : "KE=range");
      return;
    }
  } else if (request.rfind("KM=", 0) == 0 || request.rfind("KN=", 0) == 0) {
    bool isMin = (request[1] == 'N');
    const char* start = request.c_str() + 3;
    char* end;
    float grams = strtof(start, &end);
    if (end == start || *end != '\0') { sendTrimReply("KE=bad"); return; }
    if (!isSaveableCupLimit(isMin, grams)) { sendTrimReply("KE=range"); return; }
    const char* error = checkScaleChangeAllowed(false);
    if (error != nullptr) { sendTrimReply(std::string("KE=") + error); return; }
    if (!(isMin ? setMinCupGrams(grams) : setMaxCupGrams(grams))) { sendTrimReply("KE=flash"); return; }
  } else if (request.rfind("KT=", 0) == 0) {
    const char* start = request.c_str() + 3;
    char* end;
    float grams = strtof(start, &end);
    if (end == start || *end != '\0') { sendTrimReply("KE=bad"); return; }
    if (!isfinite(grams) || fabs(grams) > Dispenser::MAX_EMPTY_WEIGHT_GRAMS) { sendTrimReply("KE=range"); return; }
    const char* error = checkScaleChangeAllowed(false);
    if (error != nullptr) { sendTrimReply(std::string("KE=") + error); return; }
    if (!dispenser->setEmptyWeight(grams)) { sendTrimReply("KE=flash"); return; }
  } else {
    sendTrimReply("KE=bad");
    return;
  }
  sendTrimReply(scaleStatusReply());
}

std::string scaleStatusReply() {
  char reply[128];
  snprintf(reply, sizeof(reply), "K=%.1f,%.1f,%.1f,%d,%.1f,%.1f,%d,%.1f,%.1f,%d",
           dispenser->getAbsoluteWeight(), dispenser->getEmptyWeight(),
           dispenser->getDefaultEmptyWeight(), dispenser->isEmptyWeightSaved() ? 1 : 0,
           maxCupGrams, DEFAULT_MAX_CUP_GRAMS, isCupSettingSaved(MAX_CUP_KEY) ? 1 : 0,
           dispatcher->getMinCupGrams(), DEFAULT_MIN_CUP_GRAMS, isCupSettingSaved(MIN_CUP_KEY) ? 1 : 0);
  return reply;
}

// Cup weight window, stored next to the empty-tray weight (NVS "scale").
bool isSaveableCupLimit(bool isMin, float grams) {
  if (!isfinite(grams) || grams < CUP_LIMIT_LOW || grams > CUP_LIMIT_HIGH) {
    return false;
  }
  return isMin ? grams <= maxCupGrams - MIN_CUP_GAP_GRAMS
               : grams >= dispatcher->getMinCupGrams() + MIN_CUP_GAP_GRAMS;
}

bool isCupSettingSaved(const char* key) {
  Preferences prefs;
  bool saved = false;
  if (prefs.begin(CUP_NVS_NAMESPACE, true)) {
    saved = prefs.isKey(key);
    prefs.end();
  }
  return saved;
}

static bool saveCupSetting(const char* key, float grams) {
  Preferences prefs;
  if (!prefs.begin(CUP_NVS_NAMESPACE, false)) {
    Serial.println("[Scale] ERROR: could not open flash to save " + String(key) + ".");
    return false;
  }
  bool ok = prefs.putFloat(key, grams) == sizeof(float);
  prefs.end();
  if (!ok) {
    Serial.println("[Scale] ERROR: saving " + String(key) + " failed.");
  }
  return ok;
}

static bool eraseCupSetting(const char* key) {
  Preferences prefs;
  if (!prefs.begin(CUP_NVS_NAMESPACE, false)) {
    Serial.println("[Scale] ERROR: could not open flash to erase " + String(key) + ".");
    return false;
  }
  bool ok = !prefs.isKey(key) || prefs.remove(key);
  prefs.end();
  if (!ok) {
    Serial.println("[Scale] ERROR: erasing " + String(key) + " failed.");
  }
  return ok;
}

void loadCupLimits() {
  float minCup = DEFAULT_MIN_CUP_GRAMS;
  float maxCup = DEFAULT_MAX_CUP_GRAMS;
  Preferences prefs;
  if (prefs.begin(CUP_NVS_NAMESPACE, true)) {
    if (prefs.isKey(MIN_CUP_KEY)) minCup = prefs.getFloat(MIN_CUP_KEY, minCup);
    if (prefs.isKey(MAX_CUP_KEY)) maxCup = prefs.getFloat(MAX_CUP_KEY, maxCup);
    prefs.end();
  }
  bool valid = isfinite(minCup) && isfinite(maxCup)
            && minCup >= CUP_LIMIT_LOW && maxCup <= CUP_LIMIT_HIGH
            && minCup <= maxCup - MIN_CUP_GAP_GRAMS;
  if (!valid) {
    Serial.println("[Scale] Saved cup limits invalid. Using defaults.");
    minCup = DEFAULT_MIN_CUP_GRAMS;
    maxCup = DEFAULT_MAX_CUP_GRAMS;
  }
  dispatcher->setMinCupGrams(minCup);
  maxCupGrams = maxCup;
  Serial.println("[Scale] Cup window: " + String(minCup, 1) + "g (min, no cup below) to "
                 + String(maxCup, 1) + "g (max, not empty above)");
}

bool setMinCupGrams(float grams) {
  if (!isSaveableCupLimit(true, grams) || !saveCupSetting(MIN_CUP_KEY, grams)) return false;
  dispatcher->setMinCupGrams(grams);
  Serial.println("[Scale] Min cup weight set to " + String(grams, 1) + "g (saved).");
  return true;
}

bool setMaxCupGrams(float grams) {
  if (!isSaveableCupLimit(false, grams) || !saveCupSetting(MAX_CUP_KEY, grams)) return false;
  maxCupGrams = grams;
  Serial.println("[Scale] Max empty-cup weight set to " + String(grams, 1) + "g (saved).");
  return true;
}

bool resetMinCupGrams() {
  if (!isSaveableCupLimit(true, DEFAULT_MIN_CUP_GRAMS) || !eraseCupSetting(MIN_CUP_KEY)) return false;
  dispatcher->setMinCupGrams(DEFAULT_MIN_CUP_GRAMS);
  Serial.println("[Scale] Min cup weight reset to default " + String(DEFAULT_MIN_CUP_GRAMS, 1) + "g.");
  return true;
}

bool resetMaxCupGrams() {
  if (!isSaveableCupLimit(false, DEFAULT_MAX_CUP_GRAMS) || !eraseCupSetting(MAX_CUP_KEY)) return false;
  maxCupGrams = DEFAULT_MAX_CUP_GRAMS;
  Serial.println("[Scale] Max empty-cup weight reset to default " + String(DEFAULT_MAX_CUP_GRAMS, 1) + "g.");
  return true;
}

// Changing the empty-tray weight moves the cup thresholds, so never during a job or
// while a finished drink waits. Zeroing also needs a settled, live reading.
const char* checkScaleChangeAllowed(bool needsReading) {
  Dispatcher::DispatcherState state = dispatcher->getState();
  if (dispatcher->isServing()
      || (state != Dispatcher::DispatcherState::NO_CUP && state != Dispatcher::DispatcherState::READY)) {
    return "busy";
  }
  if (needsReading) {
    if (!dispenser->isScaleResponding()) {
      return "scale";
    }
    if (!transport->isParked() || calibrationTrayActive
        || dispenser->getState() == Dispenser::DispenserState::TARING) {
      return "busy"; // Tray moving or away (vibration, cup under a valve) or a tare in progress.
    }
  }
  return nullptr;
}

const char* checkCalibrationTrayMove() {
  if (!machineIsBooted || transport->getState() == Transport::MachineState::HOMING) {
    return "busy"; // Not homed yet.
  }
  Dispatcher::DispatcherState state = dispatcher->getState();
  if (state != Dispatcher::DispatcherState::NO_CUP && state != Dispatcher::DispatcherState::READY) {
    return "busy"; // Pouring, parking after a job, or a finished drink waiting on the tray.
  }
  return nullptr;
}

void moveTrayForCalibration(uint8_t station, bool fromBle) {
  // Never drag an open valve's stream along the rail.
  dispenser->setAllValves(Valve::Position::CLOSED);
  transport->goToStation(station);
  calibrationTrayActive = (station != 0);
  calibrationTrayTarget = station;
  calibrationTrayNotifyBle = fromBle;
  Serial.println("[Calibration] Moving tray to station " + String(station));
}

// Opening a valve pours its bottle, so the tray must be parked under it.
const char* checkValveOpenAllowed(uint8_t valveId) {
  if (transport->getState() == Transport::MachineState::MOVING_TO_TARGET_POS) {
    return "moving";
  }
  if (!transport->isAtTarget() || transport->getCurrentStationIndex() != valveId) {
    return "tray"; // Valve id n sits at rail station n.
  }
  return nullptr;
}

void endCalibrationSession() {
  dispenser->setAllValves(Valve::Position::CLOSED);
  if (calibrationTrayActive && !dispatcher->isServing()) {
    Serial.println("[Calibration] Session ended. Parking tray.");
    transport->goPark();
  }
  calibrationTrayActive = false;
  calibrationTrayTarget = -1;
}

void checkCalibrationTrayArrival() {
  if (calibrationTrayTarget < 0 || !transport->isAtTarget()) {
    return;
  }
  if (transport->getCurrentStationIndex() == calibrationTrayTarget) {
    Serial.println("[Calibration] Tray at station " + String(calibrationTrayTarget));
    if (calibrationTrayNotifyBle) {
      sendTrimReply("TM" + std::to_string(calibrationTrayTarget) + "=at");
    }
  }
  calibrationTrayTarget = -1; // Arrived, or something else moved the tray: stop waiting.
}

// Full status text for ehlo: the idle text, or what a job state means.
std::string statusTextForState() {
  const char* idle = idleStatusText();
  if (idle != nullptr) {
    return idle;
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
    ble->notifyStateIsFailed(i); // Queued; BluetoothEngine paces sends.
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
