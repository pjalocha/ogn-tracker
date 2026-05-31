#pragma once

// Pinout taken from the older FollowMe/Avionix hardware profile.

#define Reset_Pin      4   // peripheral reset: LOW active, HIGH released

#define Button_Pin    38   // user button: LOW when pushed

#define BATT_ADC_CHANNEL ADC1_CHANNEL_0 // GPIO 36 (internal battery)
#define BATT_ADC_RATIO  2.0              //
#define BATT_ADC_BIAS     0              // voltage bias [mV] (deducted after divider ratio is applied)
#define ADC_BattSense    36              // 100:100 kohm divider

// GPS: L80 with PPS and enable
#define GPS_UART UART_NUM_1  // UART for GPS
#define GPS_PinTx     17 // Tx-Data
#define GPS_PinRx     16 // Rx-Data
#define GPS_PinPPS    34 // PPS: high active
#define GPS_PinEna    33 // enable/wakeup: high active

// SX1276 RF chip
#define Radio_PinRST  -1 // no reset line
#define Radio_PinSCK  18 // SCK
#define Radio_PinMOSI 23 // MOSI
#define Radio_PinMISO 19 // MISO
#define Radio_PinCS    5 // CS
#define Radio_PinIRQ  35 // IRQ

// Status LED
#define LED_PCB_Pin    2 // status LED on the front panel

// I2C for OLED and BME280
#define I2C_PinSCL    22 // SCL
#define I2C_PinSDA    21 // SDA

// OLED
#define OLED_PinRST   -1 // no reset line

// SD card in SPI mode, using HSPI IOMUX pins
#define SD_SPI_HOST   SPI2_HOST
#define SD_PinCS      15
#define SD_PinMOSI    13
#define SD_PinSCK     14
#define SD_PinMISO    12

#define SD_SPI_DMA     2
#define SD_MOUNT_START_DELAY 1000 // [ms] let the card settle after peripheral reset/power-up
#define SD_MOUNT_RETRIES      3
#define SD_MOUNT_RETRY_DELAY 500 // [ms]
