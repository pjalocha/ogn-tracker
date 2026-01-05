#include <Arduino.h>

#ifdef WITH_NIMBLE
#include <NimBLEDevice.h>
#include <NimBLEServer.h>
#include <NimBLEUtils.h>
#include <NimBLEDescriptor.h>
#else
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#endif

#include "ble_spp.h"

// #define SERVICE_UUID        "6E400001-B5A3-F393-E0A9-E50E24DCCA9E"  // Nordic UART Service
// #define CHARACTERISTIC_UUID "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"  // 

#define SERVICE_UUID        "FFE0"  // this service is used by XCSoar and SkyDemon to get NMEA
#define CHARACTERISTIC_UUID "FFE1"

#ifdef WITH_NIMBLE
NimBLEServer* pServer = NULL;
NimBLECharacteristic* pCharacteristic = NULL;
#else
BLEServer* pServer = NULL;
BLECharacteristic* pCharacteristic = NULL;
#endif

bool BLE_SPP_isConnected = false;
const int BLE_SPP_MTU = 23;

#ifdef WITH_NIMBLE
class MyServerCallbacks: public NimBLEServerCallbacks
{ void onConnect(NimBLEServer* pServer)
  { BLE_SPP_isConnected = true;
    Serial.println("BLE device connected"); }

  void onDisconnect(NimBLEServer* pServer)
  { BLE_SPP_isConnected = false;
    Serial.println("BLE device disconnected");
    pServer->startAdvertising(); }
};
#else
class MyServerCallbacks: public BLEServerCallbacks
{ void onConnect(BLEServer* pServer)
  { BLE_SPP_isConnected = true;
    Serial.println("BLE device connected"); }

  void onDisconnect(BLEServer* pServer)
  { BLE_SPP_isConnected = false;
    Serial.println("BLE device disconnected");
    pServer->startAdvertising(); }
};
#endif

FIFO<char, 2048> BLE_SPP_RxFIFO;
FIFO<char, 2048> BLE_SPP_TxFIFO;

#ifdef WITH_NIMBLE
class MyCallbacks: public NimBLECharacteristicCallbacks
{ void onWrite(NimBLECharacteristic *pCharacteristic)
  { std::string value = pCharacteristic->getValue();
    if(value.length()<=0) return;
    for (int Idx=0; Idx<value.length(); Idx++)
    { BLE_SPP_RxFIFO.Write(value[Idx]); }
  }
};
#else
class MyCallbacks: public BLECharacteristicCallbacks
{ void onWrite(BLECharacteristic *pCharacteristic)
  { std::string value = pCharacteristic->getValue();
    if(value.length()<=0) return;
    for (int Idx=0; Idx<value.length(); Idx++)
    { BLE_SPP_RxFIFO.Write(value[Idx]); }
  }
};
#endif

#ifdef WITH_NIMBLE
void BLE_SPP_Start(const char *DevName)
{ NimBLEDevice::init(DevName);
  // Serial.printf("BLEDevice::init(%s) done\n", DevName);
  // NimBLEDevice::setPower(3);              // +3db
  // NimBLEDevice::setSecurityAuth(true, true, false); // bonding, MITM, don't need BLE secure connections as we are using passkey pairing
  // NimBLEDevice::setSecurityPasskey(123456);
  // NimBLEDevice::setSecurityIOCap(BLE_HS_IO_DISPLAY_ONLY); // Display only passkey
  pServer = NimBLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());
  // if(pServer==0) { Serial.printf("Cannot create the BLE server\n"); return; }
  NimBLEService *pService = pServer->createService(SERVICE_UUID);
  pCharacteristic = pService->createCharacteristic(
                      CHARACTERISTIC_UUID,
                      NIMBLE_PROPERTY::READ   |
                      NIMBLE_PROPERTY::WRITE  |
                      NIMBLE_PROPERTY::NOTIFY );
  pCharacteristic->setCallbacks(new MyCallbacks());
  pService->start();
  NimBLEAdvertising* pAdvertising = NimBLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->enableScanResponse(false);
  pAdvertising->start(); }
#else
void BLE_SPP_Start(const char *DevName)
{ esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT);
  delay(500);
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
  // BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  BLEAdvertising *pAdvertising = pServer->getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->setScanResponse(true);
  pAdvertising->setMinPreferred(0x06);  // Prefer 100ms connections
  pAdvertising->setMinPreferred(0x12);
  pAdvertising->start(); }
#endif

void BLE_SPP_Check(void)
{ char *Block;
  int Size=BLE_SPP_TxFIFO.getReadBlock(Block);
  if(Size==0) return;
  if(Size>BLE_SPP_MTU) Size=BLE_SPP_MTU;
  if(BLE_SPP_isConnected)
  { pCharacteristic->setValue((uint8_t *)Block, Size);
    pCharacteristic->notify(); }
  BLE_SPP_TxFIFO.flushReadBlock(Size); }
