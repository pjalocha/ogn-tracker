
#include "tft.h"

#include "ogn-radio.h"
#include "proc.h"
#include "gps.h"

#ifdef WITH_ST7735

static SPIClass TFT_SPI(1); // 0, 1, 2, VSPI, HSPI ?
       Adafruit_ST7735 TFT = Adafruit_ST7735(&TFT_SPI, TFT_PinCS, TFT_PinDC, TFT_PinRST);

void TFT_Init(void)
{ TFT_SPI.begin(TFT_PinSCK, -1, TFT_PinMOSI);
#ifdef TFT_SckFreq
  TFT_SPI.setFrequency(TFT_SckFreq);
#endif
  TFT.initR(TFT_MODEL); }

static const int TFT_BL_Chan = 0;
static const int TFT_BL_Freq = 5000;

void TFT_BL_Init(void)
{ ledcSetup(TFT_BL_Chan, TFT_BL_Freq, 8);  // set for 8-bit resolution
  ledcAttachPin(TFT_PinBL, TFT_BL_Chan); }

void TFT_BL(uint8_t Lev) { ledcWrite(TFT_BL_Chan, Lev); }

static void TFT_DrawBatt(uint16_t X, uint16_t Y, uint16_t CellSize,
                         uint16_t Cells, uint16_t Full,
                         uint16_t CellColor, uint16_t FrameColor)
{ TFT.drawRect(X, Y, CellSize+4, (CellSize+1)*Cells+3, FrameColor);   // draw the main box
  TFT.drawRect(X+2, Y-4, CellSize, 4, FrameColor);                    // draw the tip
  if(Full>Cells) Full=Cells;
  for(uint16_t Cell=0; Cell<Full; Cell++)
  { TFT.fillRect(X+2, Y+(Cells-1-Cell)*(CellSize+1)+2, CellSize, CellSize, CellColor); }
}

static void TFT_DrawBatt(uint16_t X, uint16_t Y)
{  int16_t BattVolt=(BatteryVoltage+128)>>8; // [mV] measured and averaged  battery voltage
  uint16_t Cells=5;
   int16_t Full=(BattVolt-3300+80)/160; if(Full<0) Full=0;
  uint16_t CellColor=ST77XX_GREEN;
  uint16_t FrameColor=ST77XX_WHITE;
  if(Full<=2) { CellColor=ST77XX_YELLOW; }
  if(Full<=1) { CellColor=FrameColor=ST77XX_RED; }
  static uint8_t Flip=0;
  if(BatteryVoltageRate>0x10 && Flip&1) Full++;
  TFT_DrawBatt(X, Y, 8, Cells, Full, CellColor, FrameColor);
  Flip++; }

int TFT_DrawID(bool WithAP)
{ char Line[128];
  // TFT.fillScreen(ST77XX_DARKBLUE);
  TFT.setTextColor(ST77XX_WHITE);
  TFT.setFont(&FreeMono9pt7b);
  TFT.setTextSize(1);

  int Vert=16;
  TFT.fillRect(0, Vert-11, TFT.width(), 14, ST77XX_DARKBLUE);
  TFT.setCursor(2, Vert);
  Parameters.Print(Line); Line[10]=0;
  TFT.print(Line);
  Vert+=14;
  if(Parameters.Reg[0])
  { TFT.setCursor(2, Vert);
    sprintf(Line, "Reg: %s", Parameters.Reg);
    TFT.fillRect(0, Vert-11, TFT.width(), 14, ST77XX_DARKBLUE);
    TFT.print(Line); Vert+=14; }
  if(Parameters.Pilot[0])
  { TFT.setCursor(2, Vert);
    sprintf(Line, "Plt:%s", Parameters.Pilot);
    TFT.fillRect(0, Vert-11, TFT.width(), 14, ST77XX_DARKBLUE);
    TFT.print(Line); Vert+=14; }
#ifdef WITH_AP
  if(WithAP)
  { TFT.setCursor(2, Vert);
    sprintf(Line, "AP:%s", Parameters.APname);
    TFT.fillRect(0, Vert-11, TFT.width(), 14, ST77XX_DARKBLUE);
    TFT.print(Line); Vert+=14; }
#endif
  if(Vert<60)
  { TFT.setCursor(2, Vert);
    sprintf(Line, "Bat:%5.3fV", (0.001/256)*BatteryVoltage);
    TFT.fillRect(0, Vert-11, TFT.width(), 14, ST77XX_DARKBLUE);
    TFT.print(Line); Vert+=14; }

  TFT.fillRect(0, Vert-11, TFT.width(), TFT.height()-Vert+14, ST77XX_DARKBLUE);
  uint64_t ID=getUniqueID();
  uint8_t Len=Format_String(Line, "#");
  Len+=Format_Hex(Line+Len, (uint16_t)(ID>>32));
  Len+=Format_Hex(Line+Len, (uint32_t)ID);
  Len+=Format_String(Line+Len, " v"VERSION);
  Line[Len]=0;
  TFT.setFont(0);
  TFT.setTextSize(1);
  TFT.setCursor(2, 72);
  TFT.print(Line);

  TFT_DrawBatt(146, 30);
  return 1; }

int TFT_DrawSat(void)
{ char Line[32];
  // TFT.fillScreen(ST77XX_DARKBLUE);
  TFT.setTextColor(ST77XX_WHITE);
  TFT.setFont(&FreeMono9pt7b);            // a better fitting font, but it has different vertical alignment
  TFT.setTextSize(1);

  int Vert=16;
  for(uint8_t Sys=0; Sys<=4; Sys++)      // loop over constelations
  { int Len=sprintf(Line, "%s:%d:%d", GPS_Sat::SysName(Sys), GPS_SatMon.FixSats[Sys], GPS_SatMon.VisSats[Sys]);
    uint8_t SNR=GPS_SatMon.VisSNR[Sys];  // [0.25dB]
    if(SNR>0) Len+=sprintf(Line+Len, " %4.1fdB", 0.25*SNR);
    Line[Len]=0;
    TFT.fillRect(0, Vert-11, TFT.width(), 14, ST77XX_DARKBLUE);
    // int Bar=SNR/2;                       //
    // TFT.fillRect(0, Vert-11, Bar, 14, ST77XX_DARKRED);
    TFT.setCursor(2, Vert); TFT.print(Line);
    Vert+=14; }
  TFT.fillRect(0, Vert-11, TFT.width(), 14, ST77XX_DARKBLUE);

  TFT_DrawBatt(146, 30);
  return 1; }

int TFT_DrawRF(void)
{ char Line[32];
  // TFT.fillScreen(ST77XX_DARKBLUE);
  TFT.setTextColor(ST77XX_WHITE);
  TFT.setFont(&FreeMono9pt7b);            // a better fitting font, but it has different vertical alignment
  TFT.setTextSize(1);

  int Vert=16;
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
  // Line[Len++]=' ';
  // Len+=Format_SignDec(Line+Len, (int32_t)Parameters.RFchipFreqCorr, 2, 1); // frequency correction
  // Len+=Format_String(Line+Len, "ppm");
  Line[Len]=0;
  TFT.fillRect(0, Vert-12, TFT.width(), 16, ST77XX_DARKBLUE);
  TFT.setCursor(2, Vert); TFT.print(Line); Vert+=16;

  sprintf(Line, "Rx: %+4.1fdBm", Radio_BkgRSSI);
  TFT.fillRect(0, Vert-12, TFT.width(), 16, ST77XX_DARKBLUE);
  TFT.setCursor(2, Vert); TFT.print(Line); Vert+=16;
  sprintf(Line, "Rx: %d:%d pkt", Radio_RxCount[1], Radio_RxCount[2]);
  TFT.fillRect(0, Vert-12, TFT.width(), 16, ST77XX_DARKBLUE);
  TFT.setCursor(2, Vert); TFT.print(Line); Vert+=16;

  Len=0;
  Len+=Format_String(Line+Len, Radio_FreqPlan.getPlanName());               // name of the frequency plan
  Line[Len++]=' ';
  Len+=Format_UnsDec(Line+Len, (uint32_t)(Radio_FreqPlan.getCenterFreq()/100000), 3, 1); // center frequency
  Len+=Format_String(Line+Len, "M");
  Line[Len]=0;
  TFT.fillRect(0, Vert-12, TFT.width(), 16, ST77XX_DARKBLUE);
  TFT.setCursor(2, Vert); TFT.print(Line); Vert+=16;

  TFT.fillRect(0, Vert-12, TFT.width(), 16, ST77XX_DARKBLUE);
  return 1; }

int TFT_DrawBaro(const GPS_Position *GPS)
{ char Line[32];
  // TFT.fillScreen(ST77XX_DARKBLUE);
  TFT.setTextColor(ST77XX_WHITE);
  TFT.setFont(&FreeMono9pt7b);            // a better fitting font, but it has different vertical alignment
  TFT.setTextSize(1);
  if(!GPS || !GPS->hasBaro) return 0;

  int Vert=18;
  uint8_t Len=Format_String(Line+Len, "Baro ");
  if(GPS && GPS->hasBaro)
  { Len+=Format_UnsDec(Line+Len, (GPS->Pressure+20)/40, 5, 1);
    Len+=Format_String(Line+Len, "hPa "); }
  else Len+=Format_String(Line+Len, "----.-hPa ");
  Line[Len]=0;
  TFT.fillRect(0, Vert-12, TFT.width(), 16, ST77XX_DARKBLUE);
  TFT.setCursor(2, Vert); TFT.print(Line); Vert+=16;

  Len=0;
  if(GPS && GPS->hasBaro)
  { Len+=Format_SignDec(Line+Len, (GPS->StdAltitude+5)/10, 4);
    Len+=Format_String(Line+Len, "m ");
    Len+=Format_SignDec(Line+Len, GPS->ClimbRate, 2, 1);
    Len+=Format_String(Line+Len, "m/s "); }
  else
  { Len+=Format_String(Line+Len, "----m");
    Len+=Format_String(Line+Len, " --.-m/s "); }
  Line[Len]=0;
  TFT.fillRect(0, Vert-12, TFT.width(), 16, ST77XX_DARKBLUE);
  TFT.setCursor(2, Vert); TFT.print(Line); Vert+=16;

  Len=0;
  if(GPS && GPS->hasBaro)
  { Len+=Format_SignDec(Line+Len, GPS->Temperature, 2, 1);
    Line[Len++]=0xB0;
    Line[Len++]='C';
    Line[Len++]=' ';
    Len+=Format_SignDec(Line+Len, GPS->Humidity, 2, 1);
    Line[Len++]='%'; }
  else Len+=Format_String(Line+Len, "---.-C --.-% ");
  Line[Len]=0;
  TFT.fillRect(0, Vert-12, TFT.width(), 16, ST77XX_DARKBLUE);
  TFT.setCursor(2, Vert); TFT.print(Line); Vert+=16;

  TFT.fillRect(0, Vert-12, TFT.width(), 16, ST77XX_DARKBLUE);
  if(GPS && GPS->hasBaro)
  { float Dew = DewPoint(0.1f*GPS->Temperature, 0.1f*GPS->Humidity);
    sprintf(Line, "Dew: %+5.1fC", Dew);
    Line[5]=0xB0;
    TFT.setCursor(2, Vert); TFT.print(Line);
  }

  Vert+=16;
  TFT.fillRect(0, Vert-12, TFT.width(), 16, ST77XX_DARKBLUE);
  return 1; }

#ifdef WITH_LORAWAN
int TFT_DrawLoRaWAN(const GPS_Position *GPS)
{ char Line[32];
  TFT.setTextColor(ST77XX_WHITE);
  TFT.setFont(&FreeMono9pt7b);            // a better fitting font, but it has different vertical alignment
  TFT.setTextSize(1);

  int Vert=18;
  const char *StateName[4] = { "Not-conn.", "Join-Req", "+Joined+", "Pkt-Sent" } ;
  int Len=Format_String(Line, "TTN:");
  if(WANdev.State==2) Len+=Format_Hex(Line+Len, WANdev.DevAddr);
  else if(WANdev.State<=3) Len+=Format_String(Line+Len, StateName[WANdev.State]);
                 else Len+=Format_Hex(Line+Len, WANdev.State);
  Line[Len]=0;
  TFT.fillRect(0, Vert-12, TFT.width(), 16, ST77XX_DARKBLUE);
  TFT.setCursor(2, Vert); TFT.print(Line); Vert+=16;

  if(WANdev.State>=2)
  { Len=0; // Len =Format_String(Line    , "^^");
    Len+=Format_Hex(Line+Len, (uint16_t)WANdev.UpCount);
    Len+=Format_String(Line+Len, " >> ");
    Len+=Format_Hex(Line+Len, (uint16_t)WANdev.DnCount);
    Line[Len]=0;
    TFT.fillRect(0, Vert-12, TFT.width(), 16, ST77XX_DARKBLUE);
    TFT.setCursor(2, Vert); TFT.print(Line); Vert+=16;

    // Len =Format_String(Line    , "Rx:");
    Len=0;
    Len+=Format_SignDec(Line+Len, ((int32_t)WANdev.RxSNR*10+2)>>2, 2, 1);
    Len+=Format_String(Line+Len, "dB ");
    Len+=Format_SignDec(Line+Len, (int32_t)WANdev.RxRSSI, 3);
    Len+=Format_String(Line+Len, "dBm");
    Line[Len]=0;
    TFT.fillRect(0, Vert-12, TFT.width(), 16, ST77XX_DARKBLUE);
    TFT.setCursor(2, Vert); TFT.print(Line); Vert+=16; }
  else
  { TFT.fillRect(0, Vert-12, TFT.width(), 16, ST77XX_DARKBLUE); Vert+=16;
    TFT.fillRect(0, Vert-12, TFT.width(), 16, ST77XX_DARKBLUE); Vert+=16; }

  TFT.fillRect(0, Vert-12, TFT.width(), 16, ST77XX_DARKBLUE); Vert+=16;
  TFT.fillRect(0, Vert-12, TFT.width(), 16, ST77XX_DARKBLUE); Vert+=16;

  return 1; }
#else
int TFT_DrawLoRaWAN(const GPS_Position *GPS) { return 0; }
#endif


int TFT_DrawGPS(const GPS_Position *GPS)
{ char Line[32];
  // TFT.fillScreen(ST77XX_DARKBLUE);
  TFT.setTextColor(ST77XX_WHITE);
  TFT.setFont(&FreeMono9pt7b);            // a better fitting font, but it has different vertical alignment
  TFT.setTextSize(1);

  int Vert=18;
  uint8_t Len=0;
  strcpy(Line, "--.-- --:--:--");
  if(GPS && GPS->isDateValid())
  { Format_UnsDec (Line+ 0, (uint32_t)GPS->Day,   2, 0);
    Format_UnsDec (Line+ 3, (uint32_t)GPS->Month, 2, 0);
  }
  if(GPS && GPS->isTimeValid())
  { Format_UnsDec (Line+ 6, (uint32_t)GPS->Hour,  2, 0);
    Format_UnsDec (Line+ 9, (uint32_t)GPS->Min,   2, 0);
    Format_UnsDec (Line+12, (uint32_t)GPS->Sec,   2, 0); }
  TFT.fillRect(0, Vert-12, TFT.width(), 16, ST77XX_DARKBLUE);
  TFT.setCursor(2, Vert); TFT.print(Line); Vert+=16;

  Len=0;
  // Len+=Format_String(Line+Len, "Lat: ");
  Line[Len++]=' ';
  if(GPS && GPS->isValid())
  { Len+=Format_SignDec(Line+Len,  GPS->Latitude /6, 7, 5);
    /* Line[Len++]=0xB0; */ }
  else Len+=Format_String(Line+Len, "---.-----");
  Line[Len]=0;
  TFT.fillRect(0, Vert-12, TFT.width(), 16, ST77XX_DARKBLUE);
  TFT.setCursor(2, Vert); TFT.print(Line);
  TFT.setCursor(TFT.getCursorX(), Vert-4); TFT.write('o');
  Vert+=16;

  Len=0;
  // Len+=Format_String(Line+Len, "Lon:");
  if(GPS && GPS->isValid())
  { Len+=Format_SignDec(Line+Len,  GPS->Longitude /6, 8, 5);
    /* Line[Len++]=0xB0; */ }
  else Len+=Format_String(Line+Len, "----.-----");
  Line[Len]=0;
  TFT.fillRect(0, Vert-12, TFT.width(), 16, ST77XX_DARKBLUE);
  TFT.setCursor(2, Vert); TFT.print(Line);
  TFT.setCursor(TFT.getCursorX(), Vert-4); TFT.write('o');
  Vert+=16;

  Len=0;
  // Len+=Format_String(Line+Len, "Alt: ");
  if(GPS && GPS->isValid())
  { int32_t Alt = GPS->Altitude;
    if(Alt>=0) Line[Len++]=' ';
    Len+=Format_SignDec(Line+Len,  Alt, 1, 1, 1);               // [0.1m]
  }
  else Len+=Format_String(Line+Len, "-----.-");
  Line[Len++]='m'; Line[Len++]=' ';
  Line[Len]=0;
  TFT.fillRect(0, Vert-12, TFT.width(), 16, ST77XX_DARKBLUE);
  TFT.setCursor(2, Vert); TFT.print(Line); Vert+=16;

  TFT.fillRect(0, Vert-11, TFT.width(), 16, ST77XX_DARKBLUE);
  TFT_DrawBatt(146, 30);
  return 1; }

#endif // WITH_ST7735
