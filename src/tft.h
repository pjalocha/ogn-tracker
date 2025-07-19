#include "main.h"

#ifdef WITH_ST7735

#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ST7735.h>
#include <Fonts/FreeMono9pt7b.h>

#define ST77XX_DARKORANGE  0xFC00
#define ST77XX_DARKRED     0x8000   // Dark red
#define ST77XX_DARKGREEN   0x0320   // Dark green
#define ST77XX_DARKBLUE    0x0010   // Dark blue
#define ST77XX_DARKYELLOW  0x8400   // Dark yellow/olive
#define ST77XX_DARKCYAN    0x0410   // Dark cyan
#define ST77XX_DARKMAGENTA 0x8010   // Dark magenta
#define ST77XX_DARKGRAY    0x7BEF   // Dark gray


extern Adafruit_ST7735 TFT;

void TFT_Init(void);
void TFT_BL_Init(void);
void TFT_BL(uint8_t Lev);

int TFT_DrawID(bool WithAP=0);
int TFT_DrawSat(void);
int TFT_DrawRF(void);
int TFT_DrawRFcounts(void);
int TFT_DrawBaro(const GPS_Position *GPS);
int TFT_DrawGPS(const GPS_Position *GPS);
int TFT_DrawLoRaWAN(const GPS_Position *GPS);

#endif
