#pragma once

// GPS
#define GPS_UART UART_NUM_1  // UART for GPS
#define GPS_PinTx     34 // Tx-Data
#define GPS_PinRx     33 // Rx-Data
#define GPS_PinPPS    36 // PPS
#define GPS_PinEna     3 // 35 // enable/wakeup

// SX1262 RF chip
#define Radio_PinRST  12 //
#define Radio_PinSCK   9 // SCK
#define Radio_PinMOSI 10 // MOSI
#define Radio_PinMISO 11 // MISO
#define Radio_PinCS    8 // CS
#define Radio_PinIRQ1 14 // IRQ
#define Radio_PinBusy 13 // Busy: only for SX1262

// #define I2C_PinSCL    41 // SCL
// #define I2C_PinSDA    42 // SDA

// External I2C for BME280, RTC, OLED, Magnetic sensor
#define I2C_PinSCL    18 // SCL
#define I2C_PinSDA    17 // SDA
