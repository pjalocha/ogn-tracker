#pragma once

// v1.0 https://canada1.discourse-cdn.com/free1/uploads/meshtastic/original/2X/a/a5ab6ca56101db46ad47fc5c04e7e80a457b606e.jpeg

#define Vext_PinEna    3 // put high to enable power to GPS and TFT

#define Button_Pin     0 // user button: LOW when pushed

#define BATT_ADC_CHANNEL ADC1_CHANNEL_0 // GPIO 1
#define ADC_BattSense     1 // 390:100 kohm divider
#define ADC_BattSenseEna  2 // enable battery sense by GPIO1

#define LED_PCB_Pin   18 //

// GPS
#define GPS_UART UART_NUM_1  // UART for GPS
#define GPS_PinTx     34 // Tx-Data
#define GPS_PinRx     33 // Rx-Data
#define GPS_PinPPS    36 // PPS
#define GPS_PinRst    35 // reset: not clear which way it is  active

// SX1262 RF chip
#define Radio_PinRST  12 //
#define Radio_PinSCK   9 // SCK
#define Radio_PinMOSI 10 // MOSI
#define Radio_PinMISO 11 // MISO
#define Radio_PinCS    8 // CS
#define Radio_PinIRQ1 14 // IRQ
#define Radio_PinBusy 13 // Busy: only for SX1262
#define Radio_SckFreq 8000000

// seems to have no I2C defined
// #define I2C_PinSCL    18 // SCL
// #define I2C_PinSDA    17 // SDA

// LCD
#define TFT_PinCS   38
#define TFT_PinRST  39
#define TFT_PinDC   40
#define TFT_PinSCK  41
#define TFT_PinMOSI 42
#define TFT_PinBL   21 // v1.1 - put high to activate back-light
#define TFT_Width  160
#define TFT_Height  80
#define TFT_MODEL   INITR_MINI160x80_PLUGIN
#define TFT_SckFreq 80000000
