#include "main.h"

#ifdef WITH_ST7735

#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ST7735.h>
#include <Fonts/FreeMono9pt7b.h>
#define ST77XX_DARKBLUE 0x0011

extern Adafruit_ST7735 TFT;

void TFT_Init(void);
void TFT_BL_Init(void);
void TFT_BL(uint8_t Lev);

void TFT_DrawID(bool WithAP=0);
void TFT_DrawSat(void);
void TFT_DrawRF(void);
void TFT_DrawBaro(const GPS_Position *GPS);
void TFT_DrawGPS(const GPS_Position *GPS);

#endif
