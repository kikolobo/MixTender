#pragma once

#include "Arduino.h"
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#include <BLE2902.h>

#define SERVICE_UUID "94635d24-cf8d-4ff8-9191-de713f39db89" 
#define CONTROL_UUID "4ac8a682-9736-4e5d-932b-e9b31405049c" 
#define STATUS_UUID "6bcdd021-ffa5-4522-9454-a21d025d6562"

#define DEVICE_NAME "MixTender"

class BluetoothEngine : public BLEServerCallbacks {
public:

    using DidReceiveCallback = std::function<void(const std::string&)>;
    using DidConnectCallback = std::function<void()>;
    using DidDisConnectCallback = std::function<void()>;

    BluetoothEngine();
    void setDidReceiveCallback(DidReceiveCallback callback);
    void setDidConnectCallback(DidConnectCallback callback);
    void setDidDisconnectConnectCallback(DidDisConnectCallback callback);

    void startAdvertising();
    void stopAdvertising();
    
    void sendData(std::string status);
    void notifyStatus(std::string status);    
    void notifyStateIsProcessing(uint8_t step);
    void notifyStateIsComplete(uint8_t step);
    void notifyWeightUpdate(uint8_t step, double weight);

    void heartbeat();

private:
    DidReceiveCallback didReceiveCallback;
    DidConnectCallback didConnectCallback;
    DidDisConnectCallback didDisConnectCallback;

    BLECharacteristic *characteristicControl;
    BLECharacteristic *characteristicStatus;
    
    BLEServer *server;
    bool isConnected = false;
    bool isAdvertising = false;
    std::string sentData = "";

    void setConnected(bool connected);
    void setAdvertising(bool advertising);
    void didReceiveData(std::string data);        

    class ServerCallbacks : public BLEServerCallbacks {
    public:
        BluetoothEngine *engine;
        void onConnect(BLEServer *server) override;
        void onDisconnect(BLEServer *server) override;
    };

    class StatusCallbacks : public BLECharacteristicCallbacks {
    public:
        BluetoothEngine *engine;
        void onWrite(BLECharacteristic *characteristic) override;
        void onRead(BLECharacteristic *characteristic) override;
    };

    class ControlCallbacks : public BLECharacteristicCallbacks {
    public:
        BluetoothEngine *engine;
        void onWrite(BLECharacteristic *characteristic) override;
        void onRead(BLECharacteristic *characteristic) override;
    };
};


// #pragma once

// #include "Arduino.h"
// #include <BLEDevice.h>
// #include <BLEUtils.h>
// #include <BLEServer.h>
// #include <BLE2902.h>

// #define SERVICE_UUID "94635d24-cf8d-4ff8-9191-de713f39db89" 
// #define CONTROL_UUID "4ac8a682-9736-4e5d-932b-e9b31405049c" 
// #define STATUS_UUID "6bcdd021-ffa5-4522-9454-a21d025d6562"

// #define DEVINFO_UUID (uint16_t)0x180a
// #define DEVINFO_MANUFACTURER_UUID (uint16_t)0x2a29
// #define DEVINFO_NAME_UUID (uint16_t)0x2a24
// #define DEVINFO_SERIAL_UUID (uint16_t)0x2a25

// #define DEVICE_MANUFACTURER "LoboLabs"
// #define DEVICE_NAME "MixTender"



// class BluetoothEngine: public BLEServerCallbacks
// {
//     public:
//         BluetoothEngine();
//         void startAdvertising();
//         void stopAdvertising();
//         void notifyStatus(String status);
//         void setup();
//         void hearbeat();
        
    
//     private:        
//         BLECharacteristic *characteristicControl;
//         BLECharacteristic *characteristicStatus;
//         BLEServer *server;
//         bool isConnected = false;
//         bool isAdvertising = false;
//         void setConnected(bool connected);
//         void setAdvertising(bool advertising);
//         void didReceiveData(std::string data);

//         class ServerCallbacks : public BLEServerCallbacks
//         {
//             public: 
//             BluetoothEngine *engine;
            
//             void onConnect(BLEServer *server)
//             {
//                 Serial.println("[ServerCallbacks] Connected");
//                 engine->setConnected(true);
//                 engine->setAdvertising(false);                
//             };

//             void onDisconnect(BLEServer *server)
//             {
//                 Serial.println("[ServerCallbacks] Disconnected");
//                 engine->setConnected(false);    
//             }
//         };

//         class StatusCallbacks : public BLECharacteristicCallbacks
//         {
//             public: 
//             BluetoothEngine *engine;

//             void onWrite(BLECharacteristic *characteristic)
//             {        
//                 Serial.println("[StatusCallbacks] onWrite");        
//             }

//             void onRead(BLECharacteristic *characteristic)
//             {
//                 Serial.println("[StatusCallbacks] onRead");
//             }
//         };


//         class ControlCallbacks : public BLECharacteristicCallbacks
//         {
//             public:
//             BluetoothEngine *engine;
            
//             void onWrite(BLECharacteristic *characteristic)
//             {
//                 std::string data = characteristic->getValue();                
//                 engine->didReceiveData(data);
//                 characteristic->setValue("AK");
//                 characteristic->notify();
//                 Serial.println("[ControlCallbacks] onWrite > " + String(data.c_str()));        
//             }

//             void onRead(BLECharacteristic *characteristic)
//             {        
//                 std::string data = characteristic->getValue();
//                 Serial.println("[ControlCallbacks] OnRead > " + String(data.c_str()));        
//             }
//         };
// };
