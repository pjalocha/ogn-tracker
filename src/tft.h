#include "main.h"

#if defined(WITH_ST7735) || defined(WITH_ST7789) || defined(WITH_ILI9341)

#include <SPI.h>
#include <Adafruit_GFX.h>
#if defined(WITH_ST7735)
#include <Adafruit_ST7735.h>
#elif defined(WITH_ST7789)
#include <Adafruit_ST7789.h>
#elif defined(WITH_ILI9341)
#include <Adafruit_ILI9341.h>
#define ST77XX_BLACK   ILI9341_BLACK
#define ST77XX_BLUE    ILI9341_BLUE
#define ST77XX_RED     ILI9341_RED
#define ST77XX_GREEN   ILI9341_GREEN
#define ST77XX_CYAN    ILI9341_CYAN
#define ST77XX_MAGENTA ILI9341_MAGENTA
#define ST77XX_YELLOW  ILI9341_YELLOW
#define ST77XX_WHITE   ILI9341_WHITE
#endif
#include <Fonts/FreeMono9pt7b.h>
#include <Fonts/FreeMono12pt7b.h>
#include <Fonts/FreeMonoBold12pt7b.h>
#include <Fonts/FreeMono18pt7b.h>

#define ST77XX_DARKORANGE  0xFC00
#define ST77XX_DARKRED     0x8000   // Dark red
#define ST77XX_DARKGREEN   0x0320   // Dark green
#define ST77XX_DARKBLUE    0x0010   // Dark blue
#define ST77XX_DARKYELLOW  0x8400   // Dark yellow/olive
#define ST77XX_DARKCYAN    0x0410   // Dark cyan
#define ST77XX_DARKMAGENTA 0x8010   // Dark magenta
#define ST77XX_DARKGRAY    0x7BEF   // Dark gray


#if defined(WITH_ST7735)
extern Adafruit_ST7735 TFT;
#elif defined(WITH_ST7789)
extern Adafruit_ST7789 TFT;
#elif defined(WITH_ILI9341)
extern Adafruit_ILI9341 TFT;
#endif

void TFT_Init(void);
void TFT_OFF(void);
void TFT_ON(void);
void TFT_BL_Init(void);
void TFT_BL(uint8_t Lev);
void TFT_DrawLogo(void);

int TFT_DrawID(bool WithAP=0);
int TFT_DrawLookout(void);
int TFT_DrawSatMap(void);
int TFT_DrawSat(const GPS_Position *GPS);
int TFT_DrawRF(void);
int TFT_DrawRFcounts(void);
int TFT_DrawBaro(const GPS_Position *GPS);
int TFT_DrawGPS(const GPS_Position *GPS);
int TFT_DrawLoRaWAN(const GPS_Position *GPS);
int TFT_DrawLog(const GPS_Position *GPS);

#endif
