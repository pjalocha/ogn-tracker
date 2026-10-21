#pragma once

// Seeed XIAO ESP32-S3 with the Wio-SX1262 B2B module.
// The GPS is connected to the XIAO UART RX and TX pins

#define LED_PCB_Pin 21    // but it is low-active, we need extra code here.
#define LED_PCB_Inv

#define GPS_UART UART_NUM_1
#define GPS_PinTx 43 // D6, XIAO TX -> GPS RX
#define GPS_PinRx 44 // D7, XIAO RX <- GPS TX
#define GPS_PinPPS 1  // D0, GPS PPS input

// I2C header pins for optional sensors (D4 SDA, D5 SCL).
#define I2C_PinSDA 5
#define I2C_PinSCL 6

// SX1262 on the Wio-SX1262 B2B module.
#define Radio_PinSCK   7
#define Radio_PinMISO  8
#define Radio_PinMOSI  9
#define Radio_PinCS   41
#define Radio_PinRST  42
#define Radio_PinIRQ1 39 // DIO1
#define Radio_PinBusy 40
// DIO2 (GPIO38) is configured by RadioLib as the RF switch control.
#define Radio_TCXO_Voltage 1.8
#define Radio_SckFreq 8000000
