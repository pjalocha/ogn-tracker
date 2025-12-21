#include <Arduino.h>

#include "main.h"

#include "oled.h"
#include "format.h"

#ifdef WITH_OLED

static char Line[32];

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

void OLED_DrawGPS(u8g2_t *OLED, const GPS_Position *GPS)  // GPS time, position, altitude
{ // u8g2_SetFont(OLED, u8g2_font_ncenB14_tr);
  u8g2_SetFont(OLED, u8g2_font_7x13_tf);              // 5 lines, 12 pixels/line
  uint8_t Len=0;
  if(GPS && GPS->isDateValid())
  { Format_UnsDec (Line   , (uint32_t)GPS->Day,   2, 0); Line[2]='.';
    Format_UnsDec (Line+ 3, (uint32_t)GPS->Month, 2, 0); Line[5]='.';
    Format_UnsDec (Line+ 6, (uint32_t)GPS->Year , 2, 0); Line[8]=' ';
  } else Format_String(Line, "  .  .   ");
  if(GPS && GPS->isTimeValid())
  { Format_UnsDec (Line+ 9, (uint32_t)GPS->Hour,  2, 0); Line[11]=':';
    Format_UnsDec (Line+12, (uint32_t)GPS->Min,   2, 0); Line[14]=':';
    Format_UnsDec (Line+15, (uint32_t)GPS->Sec,   2, 0);
  } else Format_String(Line+9, "  :  :  ");
  Line[17]=0;
  u8g2_DrawStr(OLED, 0, 24, Line);

  Len=0;
  Len+=Format_String(Line+Len, "Lat:  ");
  if(GPS && GPS->isValid())
  { Len+=Format_SignDec(Line+Len,  GPS->Latitude /6, 7, 5);
    Line[Len++]=0xB0; }
  else Len+=Format_String(Line+Len, "---.-----");
  Line[Len]=0;
  u8g2_DrawStr(OLED, 0, 36, Line);
  Len=0;
  Len+=Format_String(Line+Len, "Lon: ");
  if(GPS && GPS->isValid())
  { Len+=Format_SignDec(Line+Len,  GPS->Longitude /6, 8, 5);
    Line[Len++]=0xB0; }
  else Len+=Format_String(Line+Len, "----.-----");
  Line[Len]=0;
  u8g2_DrawStr(OLED, 0, 48, Line);

  // const bool isAltitudeUnitMeter=1;
  Len=0;
  Len+=Format_String(Line+Len, "Alt: ");
  if(GPS && GPS->isValid())
  { int32_t Alt = GPS->Altitude;
    if(Alt>=0) Line[Len++]=' ';
    // if(isAltitudeUnitMeter)                                       // display altitude in meters
    { Len+=Format_SignDec(Line+Len,  Alt, 1, 1, 1);               // [0.1m]
      Line[Len++]='m'; }
    // else if(isAltitudeUnitFeet)                                   // display altitude in feet
    // { Alt = (Alt*336+512)>>10;                                    // [0.1m] => [feet]
    //   Len+=Format_SignDec(Line+Len,  Alt, 1, 0, 1);               // [feet]
    //   Line[Len++]='f'; Line[Len++]='t'; }
    for( ; Len<14; ) Line[Len++]=' ';                             // tail of spaces to cover older printouts
  }
  else Len+=Format_String(Line+Len, "-----.-  ");
  Line[Len]=0;
  u8g2_DrawStr(OLED, 0, 60, Line); }


/*
void OLED_PutLine(u8g2_t *OLED, uint8_t LineIdx, const char *Line)
{ if(Line==0) return;
  // u8g2_SetFont(OLED, u8g2_font_5x8_tr);
  u8g2_SetFont(OLED, u8g2_font_amstrad_cpc_extended_8r);
  u8g2_DrawStr(OLED, 0, (LineIdx+1)*8, Line); }

void OLED_DrawPosition(u8g2_t *OLED, const GPS_Position *GPS, uint8_t LineIdx)
{ if(GPS && GPS->isValid())
  { Line[0]=' ';
    Format_SignDec(Line+1,  GPS->Latitude /60, 6, 4); Line[9]=' ';
    int32_t Alt = (GPS->Altitude+5)/10;
    if(Alt>=0) Format_UnsDec(Line+10, (uint32_t)Alt, 5, 0);
    else { Format_SignDec(Line+10, Alt, 4, 0); }
    Line[15]='m';
    OLED_PutLine(OLED, LineIdx  , Line);
    Format_SignDec(Line,    GPS->Longitude/60, 7, 4);
    Format_SignDec(Line+10, (int32_t)GPS->ClimbRate,    4, 1);
    OLED_PutLine(OLED, LineIdx+1, Line);
    Format_UnsDec (Line   , (uint32_t)GPS->Speed, 4, 1); Format_String(Line+5, "m/s  ");
    Format_UnsDec (Line+10, (uint32_t)GPS->Heading, 4, 1); Line[15]='^';
    OLED_PutLine(OLED, LineIdx+2, Line);
    Format_String(Line, "0D/00sat DOP00.0");
    Line[0]+=GPS->FixMode; Format_UnsDec(Line+3, (uint32_t)GPS->Satellites, 2);
    Format_UnsDec(Line+12, (uint32_t)GPS->HDOP, 3, 1);
    OLED_PutLine(OLED, LineIdx+3, Line);
  }
  if(GPS && GPS->isDateValid())
  { Format_UnsDec (Line   , (uint32_t)GPS->Day,   2, 0); Line[2]='.';
    Format_UnsDec (Line+ 3, (uint32_t)GPS->Month, 2, 0); Line[5]='.';
    Format_UnsDec (Line+ 6, (uint32_t)GPS->Year , 2, 0); Line[8]=' '; Line[9]=' '; }
  else Format_String(Line, "          ");
  if(GPS && GPS->isTimeValid())
  { Format_UnsDec (Line+10, (uint32_t)GPS->Hour,  2, 0);
    Format_UnsDec (Line+12, (uint32_t)GPS->Min,   2, 0);
    Format_UnsDec (Line+14, (uint32_t)GPS->Sec,   2, 0);
  } else Line[10]=0;
  OLED_PutLine(OLED, LineIdx+4, Line);
  Line[0]=0;
  if(GPS && GPS->hasBaro)
  { Format_String(Line   , "0000.0hPa 00000m");
    Format_UnsDec(Line   , GPS->Pressure/40, 5, 1);
    int32_t Alt = (GPS->StdAltitude+5)/10;
    if(Alt>=0) Format_UnsDec(Line+10, (uint32_t)Alt, 5, 0);
    else { Line[10]='-'; Format_UnsDec(Line+11, (uint32_t)(-Alt), 5, 0); }
  }
  OLED_PutLine(OLED, LineIdx+5, Line); }
*/

#endif // WITH_OLED
