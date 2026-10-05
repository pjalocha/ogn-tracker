#pragma once

#define Button_Pin    39 // user button: LOW when pushed

// GPS
#define GPS_UART UART_NUM_1  // UART for GPS
#define GPS_PinTx     17 // Tx-Data
#define GPS_PinRx     16 // Rx-Data
#define GPS_PinPPS    26 // PPS
#define GPS_PinANT     5 // antenna selection: 1=internal, 0=external

// SX1276 RF chip
#define Radio_PinRST  13 //
#define Radio_PinSCK  18 // SCK
#define Radio_PinMOSI 23 // MOSI
#define Radio_PinMISO 19 // MISO
#define Radio_PinCS   12 // CS
#define Radio_PinIRQ  35 // IRQ

#define Buzzer_Pin    25 // Beeper/buzzer
#define Buzzer_Channel 0 // LED controller channel

// I2C
#define I2C_PinSCL    21 // SCL
#define I2C_PinSDA    22 // SDA

// ILI9341 320x240 on the shared SPI bus with dedicated CS
#define TFT_PinCS   14
#define TFT_PinRST  33 // white    RST
#define TFT_PinDC   27 // grey     DC
#define TFT_PinSCK  18 // braun    SCL
#define TFT_PinMOSI 23 // black    SDA
#define TFT_PinBL   32 // magenta  BL
#define TFT_Width  320
#define TFT_Height 240
#define TFT_SckFreq 26000000
#define TFT_Rotation 0
#define TFT_MADCTL  0xA8
