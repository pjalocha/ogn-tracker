#include <Arduino.h>

#include "main.h"

#include "oled.h"
#include "format.h"

#ifdef WITH_OLED

U8G2_SH1106_128X64_NONAME_F_HW_I2C OLED(U8G2_R0, U8X8_PIN_NONE); // no reset line
// U8G2_SSD1306_128X64_NONAME_F_HW_I2C OLED(U8G2_R0, U8X8_PIN_NONE); // no reset line

void OLED_DrawLogo(u8g2_t *OLED, const GPS_Position *GPS)  // draw logo and hardware options in software
{ u8g2_DrawCircle(OLED, 96, 32, 30, U8G2_DRAW_ALL);
  u8g2_DrawCircle(OLED, 96, 32, 34, U8G2_DRAW_UPPER_RIGHT);
  u8g2_DrawCircle(OLED, 96, 32, 38, U8G2_DRAW_UPPER_RIGHT);
  // u8g2_SetFont(OLED, u8g2_font_open_iconic_all_4x_t);
  // u8g2_DrawGlyph(OLED, 64, 32, 0xF0);
  u8g2_SetFont(OLED, u8g2_font_ncenB14_tr);
  u8g2_DrawStr(OLED, 74, 31, "OGN");
  u8g2_SetFont(OLED, u8g2_font_8x13_tr);
  u8g2_DrawStr(OLED, 69, 43, "Tracker");

#ifdef WITH_GPS_MTK
  u8g2_DrawStr(OLED,  0, 28 ,"MTK GPS");
#endif
#ifdef WITH_GPS_UBX
  u8g2_DrawStr(OLED,  0, 28 ,"UBX GPS");
#endif
#ifdef WITH_SX1262
  u8g2_DrawStr(OLED,  0, 40 ,"SX1262");
#endif
#ifdef WITH_SX1276
  u8g2_DrawStr(OLED,  0, 40 ,"SX1276");
#endif
#ifdef WITH_BME280
  u8g2_DrawStr(OLED,  0, 52 ,"BME280");
#endif
}

#endif // WITH_OLED
