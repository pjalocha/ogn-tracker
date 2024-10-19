
#include "ble_spp.h"

FIFO<char, 1024> BLE_SPP_TxFIFO;

#ifdef WITH_BLE_SPP

#include <ArduinoBLE.h>

static const int BLE_MaxSize = 128;  // 20 bytes is what fits into a single BLE4 packet (but BLE5 can g up to 512 ?)

#define UART_SERVICE_UUID16                 "FFE0"
#define UART_CHARACTERISTIC_UUID16          "FFE1"

static BLEService UARTservice(UART_SERVICE_UUID16);
static BLECharacteristic UARTcharacteristic(UART_CHARACTERISTIC_UUID16,
          BLERead | BLEWriteWithoutResponse | BLENotify, BLE_MaxSize);
static BLEDescriptor UARTdescriptor("2901", "HMSoft");

static void BLE_Connect(BLEDevice Dev)
{ // Serial.printf("BLE: %s connected, %+ddBm\n", Dev.address().c_str(), Dev.rssi());
}

static void BLE_Disconnect(BLEDevice Dev)
{ // Serial.printf("BLE: %s disconnected\n", Dev.address().c_str());
}

static void BLE_UART_Written(BLEDevice Dev, BLECharacteristic Chars)
{ int Len=Chars.valueLength();
  // Serial.printf("BLE: UART[%d] <= %s\n", Len, Dev.address().c_str());
  if(Len<=0) return;
  // char Buff[BLE_MaxSize];
  // Chars.readValue(Buff, 64);
}

void BLE_SPP_Check(void)
{ BLE.poll();
  char *Data=0;
  int Len=BLE_SPP_TxFIFO.getReadBlock(Data);
  if(Len>0 && Data)
  { UARTcharacteristic.writeValue(Data, (size_t)Len);
    BLE_SPP_TxFIFO.flushReadBlock(Len); }
}

void BLE_SPP_Start(const char *DevName)
{ BLE.begin();
  BLE.setLocalName(DevName);
  BLE.setDeviceName(DevName);
  BLE.setAdvertisedService(UARTservice);
  UARTcharacteristic.setEventHandler(BLEWritten, BLE_UART_Written);
  UARTcharacteristic.addDescriptor(UARTdescriptor);
  UARTservice.addCharacteristic(UARTcharacteristic);
  BLE.addService(UARTservice);
  BLE.setEventHandler(BLEConnected,    BLE_Connect);
  BLE.setEventHandler(BLEDisconnected, BLE_Disconnect);
  BLE.advertise(); }
#endif
