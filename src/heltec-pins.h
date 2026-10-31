#pragma once

// Heltec WiFi LoRa 32 pins copied from the WITH_HELTEC mapping in
// ~/esp32-ogn-tracker/main/hal.cpp.
#define LED_PCB_Pin   25 // status LED on the PCB
#define Button_Pin     0 // user button, active low

// UBX GPS UART and PPS
#define GPS_UART UART_NUM_1
#define GPS_PinTx    13
#define GPS_PinRx    35
#define GPS_PinPPS   34

// SX1276
#define Radio_PinRST 14
#define Radio_PinSCK  5
#define Radio_PinMOSI 27
#define Radio_PinMISO 19
#define Radio_PinCS  18
#define Radio_PinIRQ 26

// OLED and I2C
#define I2C_PinSCL   15
#define I2C_PinSDA    4
#define OLED_PinRST  16
