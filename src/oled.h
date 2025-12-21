#pragma once

#ifdef WITH_OLED
#include <U8g2lib.h>

// extern U8G2_SSD1306_128X64_NONAME_F_HW_I2C OLED;
extern U8G2_SH1106_128X64_NONAME_F_HW_I2C OLED;

void OLED_DrawLogo     (u8g2_t *OLED, const GPS_Position *GPS=0);  // draw logo and hardware options in software
void OLED_DrawGPS      (u8g2_t *OLED, const GPS_Position *GPS=0);  // GPS time, position, altitude

// void OLED_DrawPosition (u8g2_t *OLED, const GPS_Position *GPS=0, uint8_t LineIdx=2); // obsolete

#endif // WITH_OLED

