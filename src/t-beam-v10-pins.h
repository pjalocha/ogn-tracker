#pragma once

// GPS
#define GPS_UART UART_NUM_1  // UART for GPS
#define GPS_PinTx     12 // Tx-Data
#define GPS_PinRx     34 // Rx-Data
#define GPS_PinPPS    37 // PPS

// SX1276 RF chip
#define Radio_PinRST  23 //
#define Radio_PinSCK   5 // SCK
#define Radio_PinMOSI 27 // MOSI
#define Radio_PinMISO 19 // MISO
#define Radio_PinCS   18 // CS
#define Radio_PinIRQ  26 // IRQ
// additional pins for SX1262
#define Radio_PinIRQ1 33 // IRQ
#define Radio_PinBusy 32 // Busy

// I2C
#define I2C_PinSCL    22 // SCL
#define I2C_PinSDA    21 // SDA

