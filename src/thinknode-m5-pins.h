#pragma once

#define Vext_PinEna    4 // put high to enable power to ?

#define Button_Pin     0 // user button: LOW when pushed
#define Button1_Pin   14
#define Button2_Pin   21

// #define BATT_ADC_CHANNEL ADC1_CHANNEL_0 // GPIO 1
// #define BATT_ADC_RATIO    5 // HTIT-Tracker has 1:4.9 voltage divider
// #define BATT_ADC_BIAS     0 // voltage bias [mV] (deducted after divider ratio is applied)
// #define ADC_BattSense     1 // 390:100 kohm divider connected to GPIO1
// #define ADC_BattSenseEna  2 // enable battery sense by GPIO2 set HIGH

#define LED_PCB_Pin   18 //

// GPS
#define GPS_UART UART_NUM_1  // UART for GPS
#define GPS_PinTx     20 // Tx-Data
#define GPS_PinRx     19 // Rx-Data
// #define GPS_PinPPS    36 // PPS
#define GPS_PinRst    13 // reset: not clear which way it is  active

// SX1262 RF chip
#define Radio_PinRST   6 //
#define Radio_PinSCK  16 // SCK
#define Radio_PinMOSI 15 // MOSI
#define Radio_PinMISO  7 // MISO
#define Radio_PinCS   17 // CS
#define Radio_PinIRQ1  4 // IRQ
#define Radio_PinBusy  5 // Busy: only for SX1262
#define Radio_SckFreq 8000000

// I2C is not defined by manufacturer, using custom pins
#define I2C_PinSDA  2
#define I2C_PinSCL  1
// #define I2C_PinSDA 48
// #define I2C_PinSCL 47
