#pragma once

// #define Button_Pin     0 // user button: LOW when pushed
#define Button1_Pin   14
#define Button2_Pin   21

#define BATT_ADC_CHANNEL ADC1_CHANNEL_7 // GPIO 8
#define BATT_ADC_RATIO    2 //
#define BATT_ADC_BIAS     0 // voltage bias [mV] (deducted after divider ratio is applied)
#define ADC_BattSense     8 //
// #define ADC_BattSenseEna 18 // V-USB sense: 12, V-GPS sense: 10, ?

// #define LED_PCB_Pin      18 //

// GPS
#define GPS_UART UART_NUM_1  // UART for GPS
#define GPS_PinTx     20 // Tx-Data towards CPU
#define GPS_PinRx     19 // Rx-Data towards GPS
// #define GPS_PinPPS    36 // PPS
#define GPS_PinRst    13 // reset: not clear which way it is  active
#define GPS_PinWake   11 // Low:sleep, High: wake
#define GPS_PinSwitch 10 // user ON/OFF of the GPS ?

// SX1262 RF chip
#define Radio_PinRST   6 //
#define Radio_PinSCK  16 // SCK
#define Radio_PinMOSI 15 // MOSI
#define Radio_PinMISO  7 // MISO
#define Radio_PinCS   17 // CS
#define Radio_PinIRQ1  4 // IRQ
#define Radio_PinBusy  5 // Busy: only for SX1262
#define Radio_PinEnable 46 // power-enable for sx1262 ?
#define Radio_SckFreq 8000000

#define Buzzer_Pin     9 // GPIO  3 = Beeper
#define Buzzer_Channel 0 // LED controller channel

// I2C is not defined by manufacturer, using custom pins
// #define I2C_PinSDA  2     // RTC at 0x51
// #define I2C_PinSCL  1
#define I2C_PinSDA 48     // PCA9557 at 0x18
#define I2C_PinSCL 47

// e-Paper display
#define EPD_PinMISO     0 // not needed, and possibly not connected
#define EPD_PinMOSI    45
#define EPD_PinSCK     38
#define EPD_PinCS      39
#define EPD_PinDC      40
#define EPD_PinRST     41
#define EPD_PinBUSY    42

// PCA9557
#define PCA_PinBLUE   1
#define PCA_PinPWR    2
#define PCA_PinRED    3
#define PCA_PinIO     4
#define PCA_PinEPD    5  // just back-light power ?
