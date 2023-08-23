#pragma once

// GPS
#define GPS_UART UART_NUM_1  // UART for GPS
#define GPS_PinTx      8 // Tx-Data
#define GPS_PinRx      9 // Rx-Data
#define GPS_PinPPS     6 // PPS
#define GPS_PinEna     7 // enable/wakeup

// SX1276 or SX1262 RF chip
#define Radio_PinRST   5 //
#define Radio_PinSCK  12 // SCK
#define Radio_PinMOSI 11 // MOSI
#define Radio_PinMISO 13 // MISO
#define Radio_PinCS   10 // CS
#define Radio_PinIRQ   1 // IRQ
#define Radio_PinBusy  4 // Busy: only for SX1262

// I2C
#define I2C_PinSCL    42 // SCL
#define I2C_PinSDA    41 // SDA

