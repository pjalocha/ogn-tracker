#pragma once

#ifdef WITH_OLED
#include <U8g2lib.h>

#ifdef WITH_BIGOLED
extern U8G2_SH1106_128X64_NONAME_F_HW_I2C OLED;
#else
extern U8G2_SSD1306_128X64_NONAME_F_HW_I2C OLED;
#endif

void OLED_DrawLogo     (u8g2_t *OLED, const GPS_Position *GPS=0);  // draw logo and hardware options in software
void OLED_DrawStatusBar(u8g2_t *OLED, const GPS_Position *GPS=0);  // status bar on top of the OLED
void OLED_DrawGPS      (u8g2_t *OLED, const GPS_Position *GPS=0);  // GNSS time, position, altitude
void OLED_DrawSatSNR   (u8g2_t *OLED, const GPS_Position *GPS=0);  // GNSS SNR
void OLED_DrawID       (u8g2_t *OLED, const GPS_Position *GPS=0);
void OLED_DrawBaro     (u8g2_t *OLED, const GPS_Position *GPS=0);
void OLED_DrawRF       (u8g2_t *OLED, const GPS_Position *GPS=0);
void OLED_DrawRelayOGN (u8g2_t *OLED, const GPS_Position *GPS=0);

#endif // WITH_OLED

