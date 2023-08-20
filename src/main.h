#pragma once

#include <Arduino.h>
#include <stdint.h>

uint64_t getUniqueMAC(void);
uint64_t getUniqueID(void);
uint32_t getUniqueAddress(void);

#ifndef VERSION
#define VERSION 0.0.0
#endif

#define HARDWARE_ID 0x04
#define SOFTWARE_ID 0x01

#define WITH_SPIFFS

#define WITH_OGN1                          // OGN protocol version 1
#define OGN_Packet OGN1_Packet

#define DEFAULT_AcftType        1         // [0..15] default aircraft-type: glider
#define DEFAULT_GeoidSepar     40         // [m]
#define DEFAULT_CONbaud    115200         // [bps]
#define DEFAULT_PPSdelay      100         // [ms]
#define DEFAULT_FreqPlan        1

#include "parameters.h"

#ifdef WITH_TBEAM10
#include "t-beam-v10-pins.h"
#endif

#ifdef WITH_TBEAM07
#include "t-beam-v07-pins.h"
#endif

extern FlashParameters Parameters;

extern SemaphoreHandle_t CONS_Mutex;
extern SemaphoreHandle_t I2C_Mutex;

extern uint8_t PowerMode;                 // 0=sleep/minimal power, 1=comprimize, 2=full power

uint8_t I2C_Restart(uint8_t Bus);
uint8_t I2C_Read (uint8_t Bus, uint8_t Addr, uint8_t Reg, uint8_t *Data, uint8_t Len, uint8_t Wait=10);
uint8_t I2C_Write(uint8_t Bus, uint8_t Addr, uint8_t Reg, uint8_t *Data, uint8_t Len, uint8_t Wait=10);

template <class Type>
 inline uint8_t I2C_Write(uint8_t Bus, uint8_t Addr, uint8_t Reg, Type &Object, uint8_t Wait=10)
{ return I2C_Write(Bus, Addr, Reg, (uint8_t *)&Object, sizeof(Type), Wait); }
template <class Type>
 inline uint8_t I2C_Read (uint8_t Bus, uint8_t Addr, uint8_t Reg, Type &Object, uint8_t Wait=10)
{ return I2C_Read (Bus, Addr, Reg, (uint8_t *)&Object, sizeof(Type), Wait); }

void LED_PCB_On   (void);                // LED on the PCB for vizual indications
void LED_PCB_Off  (void);
void LED_PCB_Flash(uint8_t Time=100);    // Flash the PCB LED for a period of [ms]

int  CONS_UART_Read       (uint8_t &Byte); // non-blocking
void CONS_UART_Write      (char     Byte); // blocking
int  CONS_UART_Free       (void);          // how many bytes can be written to the transmit buffer

int   GPS_UART_Full         (void);
int   GPS_UART_Read         (uint8_t &Byte);
void  GPS_UART_Write        (char     Byte);
void  GPS_UART_Flush        (int MaxWait  );
void  GPS_UART_SetBaudrate  (int BaudRate );
#ifdef GPS_PinPPS
bool  GPS_PPS_isOn();
#else
inline bool  GPS_PPS_isOn() { return 0; }
#endif

uint16_t BatterySense(int Samples=4); // [mV]

#ifdef WITH_SPIFFS
int  SPIFFS_Register(const char *Path="/spiffs", const char *Label="intlog", size_t MaxOpenFiles=5);
int  SPIFFS_Info(size_t &Total, size_t &Used, const char *Label="intlog");
#endif

typedef union
{ uint32_t Flags;         // hardware status
  struct
  { bool AXP192:1;        // Charge controller chip
    bool AXP202:1;        // Charge controller chip
    bool BMP280:1;        // BMP280 pressure sensor
    bool BME280:1;        // BME280 pressure sensor
    bool Radio :1;        // SX1276 or SX1262 radio TRX
    bool GPS   :1;        // GPS receiver
    bool SPIFFS:1;        // SPIFFS
  } ;
} HardItems;

extern HardItems Hardware;

