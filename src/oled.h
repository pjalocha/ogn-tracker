#pragma once

#ifdef WITH_OLED
#include <U8g2lib.h>

// extern U8G2_SSD1306_128X64_NONAME_F_HW_I2C OLED;
extern U8G2_SH1106_128X64_NONAME_F_HW_I2C OLED;

void OLED_DrawLogo     (u8g2_t *OLED, const GPS_Position *GPS);  // draw logo and hardware options in software

#endif // WITH_OLED

