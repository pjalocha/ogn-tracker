#include <Arduino.h>

#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

#include "ble_spp.h"

#define SERVICE_UUID        "FFE0"
#define CHARACTERISTIC_UUID "FFE1"

BLEServer* pServer = NULL;
BLECharacteristic* pCharacteristic = NULL;
bool BLE_SPP_isConnected = false;
const int BLE_SPP_MTU = 23;

class MyServerCallbacks: public BLEServerCallbacks
{ void onConnect(BLEServer* pServer)
  { BLE_SPP_isConnected = true;
    Serial.println("BLE device connected"); }

  void onDisconnect(BLEServer* pServer)
  { BLE_SPP_isConnected = false;
    Serial.println("BLE device disconnected");
    pServer->startAdvertising(); }
};

FIFO<char, 2048> BLE_SPP_RxFIFO;
FIFO<char, 2048> BLE_SPP_TxFIFO;

class MyCallbacks: public BLECharacteristicCallbacks
{ void onWrite(BLECharacteristic *pCharacteristic)
  { std::string value = pCharacteristic->getValue();
    if(value.length()<=0) return;
    for (int Idx=0; Idx<value.length(); Idx++)
    { BLE_SPP_RxFIFO.Write(value[Idx]); }
  }
};

void BLE_SPP_Start(const char *DevName)
{ // esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT);
  // delay(500);
  BLEDevice::init(DevName);
  // Serial.printf("BLEDevice::init(%s) done\n", DevName);
  pServer = BLEDevice::createServer();
  // if(pServer==0) { Serial.printf("Cannot create the BLE server\n"); return; }
  pServer->setCallbacks(new MyServerCallbacks());
  BLEService *pService = pServer->createService(SERVICE_UUID);
  pCharacteristic = pService->createCharacteristic(
                      CHARACTERISTIC_UUID,
                      BLECharacteristic::PROPERTY_READ   |
                      BLECharacteristic::PROPERTY_WRITE  |
                      BLECharacteristic::PROPERTY_NOTIFY
                    );
  pCharacteristic->addDescriptor(new BLE2902());
  pCharacteristic->setCallbacks(new MyCallbacks());
  pService->start();
  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->setScanResponse(true);
  pAdvertising->setMinPreferred(0x06);  // Prefer 100ms connections
  pAdvertising->setMinPreferred(0x12);
  BLEDevice::startAdvertising(); }

void BLE_SPP_Check(void)
{ char *Block;
  int Size=BLE_SPP_TxFIFO.getReadBlock(Block);
  if(Size==0) return;
  if(Size>BLE_SPP_MTU) Size=BLE_SPP_MTU;
  if(BLE_SPP_isConnected)
  { pCharacteristic->setValue((uint8_t *)Block, Size);
    pCharacteristic->notify(); }
  BLE_SPP_TxFIFO.flushReadBlock(Size); }
