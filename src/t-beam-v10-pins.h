#pragma once

#define Button_Pin    38 // user button: LOW when pushed

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

#define Flasher_Pin    4 // High intensity LED for collision alert: HIGH active

#define Buzzer_Pin    13 // Beeper/buzzer
#define Buzzer_Channel 0 // LED controller channel

// I2C
#define I2C_PinSCL    22 // SCL
#define I2C_PinSDA    21 // SDA

// OLED
#define OLED_PinRST   -1 // no reset line

// ST7789 240x240 with CS tied to GND
#define TFT_PinCS   -1
#define TFT_PinRST  33 // white    RST
#define TFT_PinDC    2 // grey     DC
#define TFT_PinSCK  13 // braun    SCL
#define TFT_PinMOSI 14 // black    SDA
#define TFT_PinBL   15 // magenta  BL
#define TFT_Width  240
#define TFT_Height 240
#define TFT_SckFreq 10000000
