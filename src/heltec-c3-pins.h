#pragma once

// GPS
#define GPS_UART UART_NUM_1  // UART for GPS
#define GPS_PinTx     21 // Tx-Data
#define GPS_PinRx     20 // Rx-Data
// #define GPS_PinPPS    -- // PPS

// SX1276 RF chip
#define Radio_PinRST   5 //
#define Radio_PinSCK  10 // SCK
#define Radio_PinMOSI  7 // MOSI
#define Radio_PinMISO  6 // MISO
#define Radio_PinCS    8 // CS
#define Radio_PinIRQ   3 // IRQ
#define Radio_PinIRQ1  3 // IRQ
#define Radio_PinBusy  4 // Busy

// I2C
#define I2C_PinSCL     9 // SCL
#define I2C_PinSDA     2 // SDA

