
#include "BluetoothEngine.h"

BluetoothEngine::BluetoothEngine() {
    Serial.println("[BluetoothEngine] Initializing");
    BLEDevice::init(DEVICE_NAME);
    // Accept up to BLE_MTU bytes when the phone negotiates (iOS asks for ~185+).
    // With the 23-byte default, messages over 20 bytes (long $0= texts) can be cut off.
    esp_err_t mtuErr = BLEDevice::setMTU(BLE_MTU);
    if (mtuErr != ESP_OK) {
        Serial.println("[BluetoothEngine] WARNING: setMTU(" + String(BLE_MTU) + ") failed: " + String(mtuErr));
    }
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

    if (!isConnected) {
        txQueue_.clear(); // Nothing to send to. A new session starts fresh (ehlo).
        return;
    }

    // Send queued messages one at a time, at least TX_MIN_INTERVAL_MS apart, in
    // order, without ever blocking loop().
    if (!txQueue_.empty() && millis() - lastTxMs_ >= TX_MIN_INTERVAL_MS) {
        std::string txString = std::move(txQueue_.front());
        txQueue_.pop_front();
        characteristicStatus->setValue(txString);
        characteristicStatus->notify();
        characteristicStatus->indicate();
        lastTxMs_ = millis();
        Serial.println("[BluetoothEngine] TXD: " + String(txString.c_str()));
    }
}

void BluetoothEngine::setConnected(bool connected) {
    isConnected = connected;
    // New session: don't suppress a message just because the last session ended
    // on it. This runs on the BLE task, so only flag it; sendData() clears it.
    resetSentData_ = true;

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

void BluetoothEngine::notifyStateIsFailed(uint8_t step) {
    sendData("S" + std::to_string(step) + "=F;");
}

void BluetoothEngine::notifyWeightUpdate(uint8_t step, double weight) {
    sendData("W" + std::to_string(step) + "=" + std::to_string(weight) + ";"); 
}

void BluetoothEngine::notifyStatus(std::string status) {
    sendData("$0=" + status);
}

void BluetoothEngine::notifyCupStatus(bool status) {
    std::string statusStr = status ? "1" : "0";
    sentData = "";
    sendData("$1=" + statusStr);
}


void BluetoothEngine::sendData(std::string txString, bool force) {  
    if (isConnected == false) {           
        return;
    }

    if (resetSentData_.exchange(false)) {
        sentData = "";
    }

    if (!force && txString == sentData) {     
        return;
    }

    if (txQueue_.size() >= TX_QUEUE_MAX) {
        Serial.println("[BluetoothEngine] TX queue full, dropping: " + String(txQueue_.front().c_str()));
        txQueue_.pop_front();
    }
    txQueue_.push_back(txString); // Sent by heartbeat().
    sentData = txString;
} 



//ServerCallbacks
void BluetoothEngine::ServerCallbacks::onConnect(BLEServer *server) {
    Serial.println("[ServerCallbacks] Connected");
    engine->setConnected(true);
    engine->setAdvertising(false);
}

void BluetoothEngine::ServerCallbacks::onMtuChanged(BLEServer *server, esp_ble_gatts_cb_param_t *param) {
    // Runs on the BLE task: log only.
    Serial.println("[ServerCallbacks] MTU negotiated: " + String(param->mtu.mtu) + " (max message " + String(param->mtu.mtu - 3) + " bytes)");
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
