
#include "BluetoothEngine.h"

BluetoothEngine::BluetoothEngine() {
    Serial.println("[BluetoothEngine] Initializing");
    BLEDevice::init(DEVICE_NAME);    
    server = BLEDevice::createServer();

    ServerCallbacks *serverCallbacks = new ServerCallbacks();
    serverCallbacks->engine = this;
    server->setCallbacks(serverCallbacks);

    BLEService *service = server->createService(SERVICE_UUID);

    // Control
    characteristicControl = service->createCharacteristic(CONTROL_UUID, BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_NOTIFY);
    ControlCallbacks *controlCallbacks = new ControlCallbacks();
    controlCallbacks->engine = this;
    characteristicControl->setCallbacks(controlCallbacks);
    characteristicControl->addDescriptor(new BLE2902());    

    // Status
    characteristicStatus = service->createCharacteristic(STATUS_UUID, BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_NOTIFY);
    StatusCallbacks *statusCallbacks = new StatusCallbacks();
    statusCallbacks->engine = this;
    characteristicStatus->setCallbacks(statusCallbacks);
    characteristicStatus->addDescriptor(new BLE2902());

    // Start Service
    service->start();

    // Setup Advertising
    BLEAdvertising *advertising = BLEDevice::getAdvertising();
    advertising->addServiceUUID(SERVICE_UUID);
    advertising->setScanResponse(true);
    advertising->setMinPreferred(0x06);
    advertising->setMinPreferred(0x12);

    startAdvertising();
}

void BluetoothEngine::setDidReceiveCallback(DidReceiveCallback callback) {
    didReceiveCallback = callback;
}

void BluetoothEngine::setDidConnectCallback(DidConnectCallback callback) {
    didConnectCallback = callback;
}

void BluetoothEngine::setDidDisconnectConnectCallback(DidDisConnectCallback callback) {
    didDisConnectCallback = callback;
}

void BluetoothEngine::startAdvertising() {
    BLEDevice::startAdvertising();
    isAdvertising = true;
}

void BluetoothEngine::stopAdvertising() {
    BLEDevice::stopAdvertising();
    isAdvertising = false;
}


void BluetoothEngine::heartbeat() {
    if (!isConnected && !isAdvertising) {
        startAdvertising();
    }
}

void BluetoothEngine::setConnected(bool connected) {
    isConnected = connected;

    if (connected == true) {
        if (didConnectCallback != nullptr) {
            didConnectCallback();
        }
    } else {
        if (didDisConnectCallback != nullptr) {
            didDisConnectCallback();
        }
    }
}

void BluetoothEngine::setAdvertising(bool advertising) {
    isAdvertising = advertising;
}

void BluetoothEngine::didReceiveData(std::string data) {
    Serial.println("[BluetoothEngine] OnWrite > " + String(data.c_str()));

    if (didReceiveCallback != nullptr) {
        didReceiveCallback(data);
    }        
}

void BluetoothEngine::notifyStateIsProcessing(uint8_t step) {
    sendData("S" + std::to_string(step) + "=P;"); 
}

void BluetoothEngine::notifyStateIsComplete(uint8_t step) {
    sendData("S" + std::to_string(step) + "=C;"); 
}

void BluetoothEngine::notifyWeightUpdate(uint8_t step, double weight) {
    sendData("W" + std::to_string(step) + "=" + std::to_string(weight) + ";"); 
}

void BluetoothEngine::notifyStatus(std::string status) {
    sendData("$0=" + status);
}


void BluetoothEngine::sendData(std::string txString) {  
    if (isConnected == false) {        
        return;
    }

    if (txString == sentData) {     
        return;
    }

    characteristicStatus->setValue(txString);
    characteristicStatus->notify();
    characteristicStatus->indicate();
    
    sentData = txString;
    Serial.println("[BluetoothEngine] TXD: " + String(txString.c_str()));
} 



//ServerCallbacks
void BluetoothEngine::ServerCallbacks::onConnect(BLEServer *server) {
    Serial.println("[ServerCallbacks] Connected");
    engine->setConnected(true);
    engine->setAdvertising(false);
}

void BluetoothEngine::ServerCallbacks::onDisconnect(BLEServer *server) {
    Serial.println("[ServerCallbacks] Disconnected");
    engine->setConnected(false);    
}

//ControlCallbacks
void BluetoothEngine::ControlCallbacks::onWrite(BLECharacteristic *characteristic) {
    std::string data = characteristic->getValue();    
    // characteristic->setValue("AK");
    // characteristic->notify();
    Serial.println("[ControlCallbacks] onWrite > " + String(data.c_str()));
    engine->didReceiveData(data);
}

void BluetoothEngine::ControlCallbacks::onRead(BLECharacteristic *characteristic) {
    std::string data = characteristic->getValue();
    Serial.println("[ControlCallbacks] OnRead > " + String(data.c_str()));
}

//StatusCallbacks
void BluetoothEngine::StatusCallbacks::onWrite(BLECharacteristic *characteristic) {
    Serial.println("[StatusCallbacks] onWrite");
}

void BluetoothEngine::StatusCallbacks::onRead(BLECharacteristic *characteristic) {
    Serial.println("[StatusCallbacks] onRead");
}
