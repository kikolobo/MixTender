#include "Arduino.h"
#include "SPI.h"
#include <TMC5160.h>
#include "HX711.h"
#include "Transport.h"
#include "Valve.h"
#include "Dispenser.h"
#include "Dispatcher.h"
#include "LedManager.h"
// #include <FastLED.h>

// #define NUM_LEDS 75
// #define DATA_PIN SDA

// CRGB leds[NUM_LEDS];


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
const float calibration_factor = -438; //-438 worked for my 440lb max scale setup

// HX711 scale;
uint8_t stationIdx = 0;

uint32_t servoAdjustValue_ = 0;

static unsigned long t_dirchange, lc_update, t_echo;

std::shared_ptr<Transport> transport;
std::shared_ptr<Dispenser> dispenser;
std::unique_ptr<Dispatcher> dispatcher;
std::unique_ptr<LedManager> ledMan;


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
  

  
  //  FastLED.addLeds<NEOPIXEL, DATA_PIN>(leds, NUM_LEDS);  // GRB ordering is assumed
   

   digitalWrite(CH1_PIN, LOW);
   digitalWrite(CH2_PIN, LOW);
   digitalWrite(CH3_PIN, LOW);

  Serial.println("# LoboLabs MixTender V1.0 - Dec 2023");
  Serial.println("[main][setup] Initializing System...");

  SPI.begin(SCK_PIN, POCI_MISO_PIN, PICO_MOSI_PIN, CS_PIN);

  transport = std::make_shared<Transport>(HOME_SW_PIN, EN_PIN, CS_PIN);
  dispenser = std::make_shared<Dispenser>(LC_DAT, LC_SCK, calibration_factor , 444.0);
  dispatcher = std::make_unique<Dispatcher>(dispenser, transport);

  transport->defineStation(12);  
  transport->defineStation(426);
  transport->defineStation(875);
  transport->defineStation(1309);
  transport->defineStation(1759);
  transport->defineStation(2192);

  transport->defineStation(650); //Pumps
  
  //Register Valves
  dispenser->registerValve(std::make_shared<Valve>(SERVO0_PIN));
  dispenser->registerValve(std::make_shared<Valve>(SERVO1_PIN));
  dispenser->registerValve(std::make_shared<Valve>(SERVO2_PIN));
  dispenser->registerValve(std::make_shared<Valve>(SERVO3_PIN));
  dispenser->registerValve(std::make_shared<Valve>(SERVO4_PIN));
  dispenser->registerValve(std::make_shared<Valve>(SERVO5_PIN));

  //Register Pumps
  dispenser->registerPump(std::make_shared<Pump>(CH1_PIN));
  dispenser->registerPump(std::make_shared<Pump>(CH2_PIN));
  dispenser->registerPump(std::make_shared<Pump>(CH3_PIN));


ledMan = std::make_unique<LedManager>(SDA);


transport->refMachine([](bool success) {    
    Serial.println("[main][refMachineCB] Machine Homed + Scale Tared");
});

Serial.println("[main][setup] Done");

ledMan->setAllLeds(CRGB(0,0,50));
  // ledMan->fadeTo(CRGB(0,0,0), CRGB(0,0,50), 1000);
}


void loop() {
  transport->heartbeat(); 
  dispenser->heartbeat();
  dispatcher->heartbeat();
  ledMan->heartbeat();


  ledMan->trackTray(transport->getCurrentPosition(), CRGB(0,255,255), CRGB(0,0,20));



  uint32_t now = millis(); 
    
  if (Serial.available() > 0) {  // Check if data is available to read
    auto receivedChar = Serial.read(); // Read the incoming byte
    // Serial.print((int)receivedChar);
    switch (receivedChar) {       
     case '?':
        Serial.println(transport->getCurrentPosition());
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
        // dispatcher->addStep(1, 50.0-(50 *.08));   
        // dispatcher->addStep(2, 50.0-(50 *.08));   
        // dispatcher->addStep(3, 100.0-(100 *.08));   
        dispatcher->addStep(Dispenser::DispenseType::VALVE, 4, 4, 50.0);
        dispatcher->addStep(Dispenser::DispenseType::VALVE, 6, 6, 50.0);
        dispatcher->addStep(Dispenser::DispenseType::PUMP, 1, 7, 50.0);  //Pump 1 / Station 7
        // dispatcher->addStep(6, 50.0-(50 *.08));   
        // dispatcher->addStep(2, 50.0-(50 *.08));
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
      
      default:        
        break;
    }    
        
  }
      
}
