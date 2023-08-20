#pragma once

// GPS
#define GPS_UART UART_NUM_1  // UART for GPS
#define GPS_PinTx     15 // Tx-Data
#define GPS_PinRx     12 // Rx-Data

// SX1276 or SX1262 RF chip
#define Radio_PinRST  23 //
#define Radio_PinSCK   5 // SCK
#define Radio_PinMOSI 27 // MOSI
#define Radio_PinMISO 19 // MISO
#define Radio_PinCS   18 // CS
#define Radio_PinIRQ  26 // IRQ
#define Radio_PinBusy 32 // Busy but only for SX1262

// I2C
#define I2C_PinSCL    22 // GPIO  1 = SCL
#define I2C_PinSDA    21 // GPIO  2 = SDA

