#include <Arduino.h>

#include "main.h"
#include "ble_spp.h"
#include "oled.h"

#include "gps.h"
#include "proc.h"
#include "format.h"

#include "ogn-radio.h"

#ifdef WITH_OLED

static char Line[32];

#ifdef WITH_BIGOLED
U8G2_SH1106_128X64_NONAME_F_HW_I2C OLED(U8G2_R0, OLED_PinRST);
#else
U8G2_SSD1306_128X64_NONAME_F_HW_I2C OLED(U8G2_R0, OLED_PinRST);
#endif

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

static int8_t BattCapacity(uint16_t mVolt) // deduce battery capacity from its voltage
{ if(mVolt>=4100) return 100;              // if 4.1V or more then full
  if(mVolt<=1000) return  -1;              // if below 1.0V then no-battery
  if(mVolt<=3600) return   0;              // if below 3.6V then empty
  return (mVolt-3600+2)/5; }               // otherwise a linear function from 3.6V to 4.1V

void OLED_DrawStatusBar(u8g2_t *OLED, const GPS_Position *GPS)   // status bar on top of the OLED
{ static bool Odd=0;
  int8_t Cap = BattCapacity(BatteryVoltage>>8);           // [%] est. battery capacity
  uint8_t BattLev = (Cap+10)/20;                          // [0..5] convert to display scale
  uint8_t Charging = BatteryVoltageRate>0;                // charging or not changing ?
  static uint8_t DispLev = 0;
  if(Charging==1 || Charging==2) { DispLev++; if(DispLev>5) DispLev = BattLev?BattLev-1:0; }
                           else  { DispLev = BattLev; }
  if(Cap>=0)
  { if(BattLev==0 && !Charging && Odd)                // when battery is empty, then flash it at 0.5Hz
    { }                                               // thus here avoid printing the battery symbol for flashing effect
    else                                              // print the battery symbol with DispLev
    { u8g2_SetFont(OLED, u8g2_font_battery19_tn);
      u8g2_SetFontDirection(OLED, 3);
      u8g2_DrawGlyph(OLED, 20, 10, '0'+DispLev);
      u8g2_SetFontDirection(OLED, 0); }
    Odd=!Odd; }

#ifdef WITH_SD
  if(SD_isMounted())
  { u8g2_SetFont(OLED, u8g2_font_twelvedings_t_all);
    u8g2_DrawGlyph(OLED, 24, 12, 0x73); }
#endif
#ifdef WITH_BLE_SPP
  if(BLE_SPP_isConnected)
  { u8g2_SetFont(OLED, u8g2_font_open_iconic_all_1x_t);
    u8g2_DrawGlyph(OLED, 36, 11, 0x5E); } // 0x4A
#endif
#ifdef WITH_WIFI
  if(WIFI_isConnected())
  { u8g2_SetFont(OLED, u8g2_font_open_iconic_all_1x_t);
    u8g2_DrawGlyph(OLED, 43, 11, 0x119); } // 0x50
#endif
#ifdef WITH_AP
  if(WIFI_isAP())
  { u8g2_SetFont(OLED, u8g2_font_open_iconic_all_1x_t);
    u8g2_DrawGlyph(OLED, 43, 11, 0xF8); } // 0x50
#endif

  static uint8_t Sec=0;
  u8g2_SetFont(OLED, u8g2_font_6x12_tr);
  strcpy(Line, "--sat --:--Z");
  if(GPS && GPS->isTimeValid())
  { Format_UnsDec (Line+6, (uint32_t)GPS->Hour,  2, 0); Line[8]=':';
    Format_UnsDec (Line+9, (uint32_t)GPS->Min,   2, 0);
  } else Format_String(Line+6, "--:--");
  if(GPS)
  { if(Sec)
    { Format_UnsDec(Line, (uint32_t)GPS->Satellites,  2); memcpy(Line+2, "sat", 3); }
    else
    { Format_UnsDec(Line, (uint32_t)(GPS_SatSNR+2)/4,  2); memcpy(Line+2, "dB ", 3);}
  }
  else Format_String(Line, "--sat");
  u8g2_DrawStr(OLED, 52, 10, Line);
  Sec++; if(Sec>=3) Sec=0; }

void OLED_DrawSatSNR(u8g2_t *OLED, const GPS_Position *GPS)
{ char Line[32];

  // u8g2_SetFont(OLED, u8g2_font_ncenB14_tr);
  u8g2_SetFont(OLED, u8g2_font_7x13_tf);              // 5 lines, 12 pixels/line

  int Vert=24;
  for(uint8_t Sys=1; Sys<=4; Sys++)
  { int Len=sprintf(Line, "%s:%d:%d", GPS_Sat::SysName(Sys), GPS_SatMon.FixSats[Sys], GPS_SatMon.VisSats[Sys]);
    uint8_t SNR=GPS_SatMon.VisSNR[Sys];
    if(SNR>0) Len+=sprintf(Line+Len, " %4.1fdB", 0.25*SNR);
         // else Len+=sprintf(Line+Len, " --.-dB");
    Line[Len]=0;
    u8g2_DrawStr(OLED, 0, Vert, Line);
    Vert+=12; }
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

void OLED_DrawID(u8g2_t *OLED, const GPS_Position *GPS)
{ char Line[128];
  u8g2_SetFont(OLED, u8g2_font_9x15_tr);
  Parameters.Print(Line); Line[10]=0;
  u8g2_DrawStr(OLED, 26, 25, Line);
  // u8g2_SetFont(OLED, u8g2_font_10x20_tr);
  u8g2_SetFont(OLED, u8g2_font_7x13_tf);
  u8g2_DrawStr(OLED, 0, 24, "ID:");
  if(Parameters.Pilot[0] || Parameters.Reg[0])
  { strcpy(Line, "Pilot: "); strcat(Line, Parameters.Pilot);
    u8g2_DrawStr(OLED, 0, 37, Line);
    strcpy(Line, "Reg: "); strcat(Line, Parameters.Reg);
    u8g2_DrawStr(OLED, 0, 49, Line); }
  else
  { u8g2_DrawStr(OLED, 20, 37, "OGN-Tracker");
    u8g2_DrawStr(OLED,  0, 49, "(c) Pawel Jalocha"); }
 u8g2_SetFont(OLED, u8g2_font_6x12_tr);
  uint64_t ID=getUniqueID();
  uint8_t Len=Format_String(Line, "#");
  Len+=Format_Hex(Line+Len, (uint16_t)(ID>>32));
  Len+=Format_Hex(Line+Len, (uint32_t)ID);
  // Line[Len++]=' ';
  // Line[Len++]='v';
    Len+=Format_String(Line+Len," " VERSION);
  Line[Len]=0;
  u8g2_DrawStr(OLED, 0, 62, Line); }

void OLED_DrawBaro(u8g2_t *OLED, const GPS_Position *GPS)
{ char Line[32];
  u8g2_SetFont(OLED, u8g2_font_7x13_tf);              // 5 lines, 12 pixels/line
  uint8_t Len=Format_String(Line+Len, "BME280 ");
  if(GPS && GPS->hasBaro)
  { Len+=Format_UnsDec(Line+Len, GPS->Pressure/4, 5, 2);
    Len+=Format_String(Line+Len, "hPa "); }
  else Len+=Format_String(Line+Len, "----.--hPa ");
  Line[Len]=0;
  u8g2_DrawStr(OLED, 0, 24, Line);
  Len=0;
  if(GPS && GPS->hasBaro)
  { Len+=Format_SignDec(Line+Len, GPS->StdAltitude, 5, 1);
    Len+=Format_String(Line+Len, "m ");
    Len+=Format_SignDec(Line+Len, GPS->ClimbRate, 2, 1);
    Len+=Format_String(Line+Len, "m/s "); }
  else
  { Len+=Format_String(Line+Len, "-----.-m");
    Len+=Format_String(Line+Len, " --.-m/s "); }
  Line[Len]=0;
  u8g2_DrawStr(OLED, 0, 36, Line);
  Len=0;
  if(GPS && GPS->hasBaro)
  { Len+=Format_SignDec(Line+Len, GPS->Temperature, 2, 1);
    Line[Len++]=0xB0;
    Line[Len++]='C';
    Line[Len++]=' ';
    Len+=Format_SignDec(Line+Len, GPS->Humidity, 2, 1);
    Line[Len++]='%'; }
  else Len+=Format_String(Line+Len, "---.- C --.-% ");
  Line[Len]=0;
  u8g2_DrawStr(OLED, 0, 48, Line);
  if(GPS && GPS->hasBaro)
  { float Dew = DewPoint(0.1f*GPS->Temperature, 0.1f*GPS->Humidity);
    sprintf(Line, "%+5.1f C dew point", Dew);
    Line[5]=0xB0;
    u8g2_DrawStr(OLED, 0, 60, Line); }
}

void OLED_DrawRF(u8g2_t *OLED, const GPS_Position *GPS) // RF 868MHz
{
  char Line[32];
  u8g2_SetFont(OLED, u8g2_font_7x13_tf);            // 5 lines. 12 pixels/line
  uint8_t Len=0;
#ifdef WITH_SX1262
  Len+=Format_String(Line+Len, "SX1262");
#endif
#ifdef WITH_SX1276
  Len+=Format_String(Line+Len, "SX1276");
#endif
  Line[Len++]=':';
  Len+=Format_SignDec(Line+Len, (int16_t)Parameters.TxPower);              // Tx power
  Len+=Format_String(Line+Len, "dBm");
  Line[Len++]=' ';
  Len+=Format_SignDec(Line+Len, (int32_t)Parameters.RFchipFreqCorr, 2, 1); // frequency correction
  Len+=Format_String(Line+Len, "ppm");
  Line[Len]=0;
  u8g2_DrawStr(OLED, 0, 24, Line);
  sprintf(Line, "Rx: %+4.1fdBm", Radio_BkgRSSI);
  u8g2_DrawStr(OLED, 0, 36, Line);
  uint32_t Sum=0;
  for(int Idx=0; Idx<8; Idx++)
    Sum+=Radio_RxCount[Idx];
  sprintf(Line, "Rx: %d pkts", Sum);
  u8g2_DrawStr(OLED, 0, 48, Line);
  Len=0;
  Len+=Format_String(Line+Len, Radio_FreqPlan.getPlanName());               // name of the frequency plan
  Line[Len++]=' ';
  Len+=Format_UnsDec(Line+Len, (uint32_t)(Radio_FreqPlan.getCenterFreq()/100000), 3, 1); // center frequency
  Len+=Format_String(Line+Len, "MHz");
  Line[Len]=0;
  u8g2_DrawStr(OLED, 0, 60, Line); }


#endif // WITH_OLED
