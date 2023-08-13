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
#define DEFAULT_PPSdelay      150         // [ms]
#define DEFAULT_FreqPlan        1

#include "parameters.h"

extern FlashParameters Parameters;

extern SemaphoreHandle_t CONS_Mutex;
extern SemaphoreHandle_t I2C_Mutex;

extern uint8_t PowerMode;                 // 0=sleep/minimal power, 1=comprimize, 2=full power

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
bool  GPS_PPS_isOn();

uint16_t BatterySense(int Samples=4); // [mV]

#ifdef WITH_SPIFFS
int  SPIFFS_Register(const char *Path="/spiffs", const char *Label="intlog", size_t MaxOpenFiles=5);
int  SPIFFS_Info(size_t &Total, size_t &Used, const char *Label="intlog");
#endif

