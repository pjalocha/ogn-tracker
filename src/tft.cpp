
#include "tft.h"

#include "ogn-radio.h"
#include "proc.h"
#include "gps.h"
#include "log.h"

#if defined(WITH_ST7789) && CONFIG_IDF_TARGET_ESP32
#include "esp32/rom/tjpgd.h"
#endif

#if defined(WITH_ST7735) || defined(WITH_ST7789) || defined(WITH_ILI9341)

#if defined(WITH_ST7789) || defined(WITH_ILI9341)
static const GFXfont *TFT_FontMain = &FreeMono12pt7b;
static const uint8_t  TFT_FontMainSize = 1;
static const int      TFT_LineVert = 20;
static const int      TFT_LineVertTight = 19;
static const int      TFT_LineFillOfs = 15;
static const int      TFT_LineFillHeight = 22;
static const int      TFT_TopVert = 20;
static const int      TFT_Footer1 = 222;
static const int      TFT_Footer2 = 234;
static const int      TFT_BattY = 38;
static const uint16_t TFT_BattCellSize = 12;
#else
static const GFXfont *TFT_FontMain = &FreeMono9pt7b;
static const uint8_t  TFT_FontMainSize = 1;
static const int      TFT_LineVert = 16;
static const int      TFT_LineVertTight = 15;
static const int      TFT_LineFillOfs = 12;
static const int      TFT_LineFillHeight = 16;
static const int      TFT_TopVert = 16;
static const int      TFT_Footer1 = 63;
static const int      TFT_Footer2 = 72;
static const int      TFT_BattY = 30;
static const uint16_t TFT_BattCellSize = 8;
#endif

static inline void TFT_SetMainFont(void)
{ TFT.setFont(TFT_FontMain);
  TFT.setTextSize(TFT_FontMainSize); }

static inline void TFT_ClearTextLine(int Vert)
{ TFT.fillRect(0, Vert-TFT_LineFillOfs, TFT.width(), TFT_LineFillHeight, ST77XX_DARKBLUE); }

#if (defined(WITH_ST7735) || defined(WITH_ST7789)) && CONFIG_IDF_TARGET_ESP32
static SPIClass TFT_SPI(HSPI);
#elif defined(WITH_ST7735) || defined(WITH_ST7789)
static SPIClass TFT_SPI(1); // works on newer ESP32 variants used by the other TFT targets
#endif
#if defined(WITH_ST7735)
       Adafruit_ST7735 TFT = Adafruit_ST7735(&TFT_SPI, TFT_PinCS, TFT_PinDC, TFT_PinRST);
#elif defined(WITH_ST7789)
       Adafruit_ST7789 TFT = Adafruit_ST7789(&TFT_SPI, TFT_PinCS, TFT_PinDC, TFT_PinRST);
#elif defined(WITH_ILI9341)
       Adafruit_ILI9341 TFT = Adafruit_ILI9341(TFT_PinCS, TFT_PinDC, TFT_PinRST);
#endif

void TFT_Init(void)
{
#if defined(WITH_ILI9341)
#ifdef Radio_PinMISO
  SPI.begin(TFT_PinSCK, Radio_PinMISO, TFT_PinMOSI);
#else
  SPI.begin(TFT_PinSCK, -1, TFT_PinMOSI);
#endif
#else
  TFT_SPI.begin(TFT_PinSCK, -1, TFT_PinMOSI);
#endif
#if defined(WITH_ST7789)
  pinMode(TFT_PinDC, OUTPUT);
  digitalWrite(TFT_PinDC, HIGH);
  if(TFT_PinRST>=0)
  { pinMode(TFT_PinRST, OUTPUT);
    digitalWrite(TFT_PinRST, HIGH);
    delay(20);
    digitalWrite(TFT_PinRST, LOW);
    delay(20);
    digitalWrite(TFT_PinRST, HIGH);
    delay(200); }
#endif
#ifdef TFT_SckFreq
#if defined(WITH_ILI9341)
  SPI.setFrequency(TFT_SckFreq);
#else
  TFT_SPI.setFrequency(TFT_SckFreq);
#endif
#endif
#if defined(WITH_ST7735)
  TFT.initR(TFT_MODEL);
#elif defined(WITH_ST7789)
  TFT.init(TFT_Width, TFT_Height, SPI_MODE3);
  TFT.setSPISpeed(TFT_SckFreq);
  TFT.enableDisplay(true);
  TFT.enableTearing(false);
  TFT.invertDisplay(true);
#elif defined(WITH_ILI9341)
  TFT.begin(TFT_SckFreq);
  TFT.invertDisplay(false);
#endif
}

const uint8_t ST7735_DispOFF  = 0x28;
const uint8_t ST7735_DispON   = 0x29;
const uint8_t ST7735_SleepIn  = 0x10;
const uint8_t ST7735_SleepOut = 0x11;

void TFT_OFF(void)
{ TFT.writeCommand(ST7735_DispOFF);
  TFT.writeCommand(ST7735_SleepIn);
  delay(5); }

void TFT_ON(void)
{ TFT.writeCommand(ST7735_SleepOut);
  delay(120);
  TFT.writeCommand(ST7735_DispON); }

static const int TFT_BL_Chan = 0;
static const int TFT_BL_Freq = 5000;

void TFT_BL_Init(void)
{
#if defined(WITH_ST7789) || defined(WITH_ILI9341)
  pinMode(TFT_PinBL, OUTPUT);
  digitalWrite(TFT_PinBL, LOW);
#else
  ledcSetup(TFT_BL_Chan, TFT_BL_Freq, 8);  // set for 8-bit resolution
  ledcAttachPin(TFT_PinBL, TFT_BL_Chan);
#endif
}

void TFT_BL(uint8_t Lev)
{
#if defined(WITH_ST7789) || defined(WITH_ILI9341)
  digitalWrite(TFT_PinBL, Lev ? HIGH : LOW);
#else
  ledcWrite(TFT_BL_Chan, Lev);
#endif
}

#if defined(WITH_ST7789) && CONFIG_IDF_TARGET_ESP32
typedef struct
{ int16_t         xpos;
  int16_t         ypos;
  const uint8_t * Input;
  uint32_t        InpSize;
  uint32_t        InpPtr;
} TFT_JPGIODEV;

static UINT TFT_jpg_input(JDEC *jd, BYTE *buff, UINT nd)
{ TFT_JPGIODEV *dev = (TFT_JPGIODEV *)jd->device;
  if(!dev->Input) return 0;
  if(dev->InpPtr >= dev->InpSize) return 0;
  if((dev->InpPtr+nd) > dev->InpSize) nd = dev->InpSize-dev->InpPtr;
  if(buff) memcpy(buff, dev->Input + dev->InpPtr, nd);
  dev->InpPtr += nd;
  return nd; }

static UINT TFT_jpg_output(JDEC *jd, void *bitmap, JRECT *rect)
{ TFT_JPGIODEV *dev = (TFT_JPGIODEV *)jd->device;
  const int16_t x = dev->xpos + rect->left;
  const int16_t y = dev->ypos + rect->top;
  const int16_t w = rect->right  - rect->left + 1;
  const int16_t h = rect->bottom - rect->top  + 1;
  uint8_t *src = (uint8_t *)bitmap;
  uint16_t rgb565[16*16];
  int pixels = w*h;
  if(pixels > (int)(sizeof(rgb565)/sizeof(rgb565[0]))) return 0;
  for(int idx=0; idx<pixels; idx++)
  { uint8_t red   = src[3*idx+0];
    uint8_t green = src[3*idx+1];
    uint8_t blue  = src[3*idx+2];
    rgb565[idx] = ((red & 0xF8)<<8) | ((green & 0xFC)<<3) | (blue>>3); }
  TFT.drawRGBBitmap(x, y, rgb565, w, h);
  return 1; }
#endif

void TFT_DrawLogo(void)
{
#if defined(WITH_ST7789) && CONFIG_IDF_TARGET_ESP32
  extern const uint8_t OGN_logo_jpg[] asm("_binary_src_OGN_logo_240x240_jpg_start");
  extern const uint8_t OGN_logo_end[] asm("_binary_src_OGN_logo_240x240_jpg_end");
  const int OGN_logo_size = OGN_logo_end-OGN_logo_jpg;
  TFT_JPGIODEV dev;
  JDEC decoder;
  const UINT WorkSize = 3800;
  char *Work = (char *)malloc(WorkSize);
  if(!Work) return;
  dev.Input = OGN_logo_jpg;
  dev.InpSize = OGN_logo_size;
  dev.InpPtr = 0;
  dev.xpos = (TFT.width()-240)/2;
  dev.ypos = (TFT.height()-240)/2;
  if(jd_prepare(&decoder, TFT_jpg_input, (void *)Work, WorkSize, &dev)==JDR_OK)
    jd_decomp(&decoder, TFT_jpg_output, 0);
  free(Work);
#elif defined(WITH_ST7735)
  TFT.fillScreen(ST77XX_WHITE);
  TFT.drawCircle(108, 34, 28, ST77XX_BLUE);
  TFT.drawCircle(108, 34, 32, ST77XX_BLUE);
  TFT.drawCircle(108, 34, 36, ST77XX_BLUE);
  TFT.setTextColor(ST77XX_BLACK);
  TFT.setFont(&FreeMono18pt7b);
  TFT.setTextSize(1);
  TFT.setCursor(52, 38);
  TFT.print("OGN");
  TFT.setFont(&FreeMonoBold12pt7b);
  TFT.setTextSize(1);
  TFT.setCursor(38, 62);
  TFT.print("Tracker");
#endif
}

static void TFT_DrawBatt(uint16_t X, uint16_t Y, uint16_t CellSize,
                         uint16_t Cells, uint16_t Full,
                         uint16_t CellColor, uint16_t FrameColor)
{ TFT.drawRect(X, Y, CellSize+4, (CellSize+1)*Cells+3, FrameColor);   // draw the main box
  TFT.drawRect(X+2, Y-4, CellSize, 4, FrameColor);                    // draw the tip
  if(Full>Cells) Full=Cells;
  // Draw alternating cells first to reduce the visible top-to-bottom wipe.
  for(uint16_t Pass=0; Pass<2; Pass++)
  { for(uint16_t Cell=0; Cell<Full; Cell++)
    { if((Cell&1)!=Pass) continue;
      TFT.fillRect(X+2, Y+(Cells-1-Cell)*(CellSize+1)+2, CellSize, CellSize, CellColor); }
  }
}

static void TFT_DrawBatt(uint16_t X, uint16_t Y)
{  int16_t BattVolt=(BatteryVoltage+128)>>8; // [mV] measured and averaged  battery voltage
  // int16_t Full=(BattVolt-3300+80)/160; if(Full<0) Full=0;
   int16_t Full=(BattVolt-3500+64)>>7; if(Full<0) Full=0;
  const uint16_t Cells=5;
  uint16_t CellColor=ST77XX_GREEN;
  uint16_t FrameColor=ST77XX_WHITE;
  if(Full<2) { CellColor=ST77XX_YELLOW; }
  if(Full<1) { CellColor=FrameColor=ST77XX_RED; }
  static uint8_t Flip=0;
  if(BatteryVoltageRate>0 && Flip&1) Full++;
  TFT_DrawBatt(X, Y, TFT_BattCellSize, Cells, Full, CellColor, FrameColor);
  Flip++; }

static uint16_t TFT_BattX(void)
{ uint16_t BattWidth = TFT_BattCellSize+4;
  if(TFT.width()<BattWidth) return 0;
  return TFT.width()-BattWidth; }

/*
static char AddrTypeChar(uint8_t AddrType)
{ if(AddrType==3) return 'O';
  if(AddrType==2) return 'F';
  if(AddrType==1) return 'I';
  return 'R'; }

static const char *AcftTypeName(uint8_t AcftType)
{ const char *TypeName[16] = { "----", "Glid", "Tow ", "Heli",
                               "SkyD", "Drop", "Hang", "Para",
                               "Pwrd", "Jet ", "UFO ", "Ball",
                               "Zepp", "UAV ", "Car ", "Fix " } ;
  if(AcftType<16) return TypeName[AcftType];
  return TypeName[0]; }
*/

int TFT_DrawID(bool WithAP)
{ char Line[32];
  // TFT.fillScreen(ST77XX_DARKBLUE);
  TFT.setTextColor(ST77XX_WHITE);
  TFT_SetMainFont();

  int Vert=TFT_TopVert;
  sprintf(Line, "%s:%c:%06X", Parameters.AcftTypeName(), Parameters.AddrTypeChar(), Parameters.Address);
  TFT_ClearTextLine(Vert);
  TFT.setCursor(2, Vert); TFT.print(Line); Vert+=TFT_LineVertTight;
#ifdef WITH_BLE_SPP
  if(Parameters.BTname[0])
  { sprintf(Line, "BT:%s", Parameters.BTname);
    TFT_ClearTextLine(Vert);
    TFT.setCursor(2, Vert); TFT.print(Line); Vert+=TFT_LineVertTight; }
#endif
#ifdef WITH_AP
  if(WithAP)
  { sprintf(Line, "AP:%s", Parameters.APname);
    TFT_ClearTextLine(Vert);
    TFT.setCursor(2, Vert); TFT.print(Line); Vert+=TFT_LineVertTight; }
#endif
  if(Parameters.Reg[0])
  { sprintf(Line, "Reg:%s", Parameters.Reg);
    TFT_ClearTextLine(Vert);
    TFT.setCursor(2, Vert); TFT.print(Line); Vert+=TFT_LineVertTight; }
  if(Parameters.Manuf[0] && Vert<TFT.height())
  { sprintf(Line, "Manuf:%s", Parameters.Manuf);
    TFT_ClearTextLine(Vert);
    TFT.setCursor(2, Vert); TFT.print(Line); Vert+=TFT_LineVertTight; }
  if(Parameters.Model[0] && Vert<TFT.height())
  { sprintf(Line, "Model:%s", Parameters.Model);
    TFT_ClearTextLine(Vert);
    TFT.setCursor(2, Vert); TFT.print(Line); Vert+=TFT_LineVertTight; }
  if(Parameters.Pilot[0] && Vert<TFT.height())
  { sprintf(Line, "Pilot:%s", Parameters.Pilot);
    TFT_ClearTextLine(Vert);
    TFT.setCursor(2, Vert); TFT.print(Line); Vert+=TFT_LineVertTight; }
  if(Parameters.Base[0] && Vert<TFT.height())
  { sprintf(Line, "Base:%s", Parameters.Base);
    TFT_ClearTextLine(Vert);
    TFT.setCursor(2, Vert); TFT.print(Line); Vert+=TFT_LineVertTight; }
  if(Parameters.Crew[0] && Vert<TFT.height())
  { sprintf(Line, "Crew:%s", Parameters.Crew);
    TFT_ClearTextLine(Vert);
    TFT.setCursor(2, Vert); TFT.print(Line); Vert+=TFT_LineVertTight; }
  if(Parameters.Task[0] && Vert<TFT.height())
  { sprintf(Line, "Task:%s", Parameters.Task);
    TFT_ClearTextLine(Vert);
    TFT.setCursor(2, Vert); TFT.print(Line); Vert+=TFT_LineVertTight; }
  if(Vert<TFT.height())
  { sprintf(Line, "Bat:%5.3fV", (0.001/256)*BatteryVoltage);
    TFT_ClearTextLine(Vert);
    TFT.setCursor(2, Vert); TFT.print(Line); Vert+=TFT_LineVertTight; }

  TFT.fillRect(0, Vert-TFT_LineFillOfs, TFT.width(), TFT.height()-Vert+TFT_LineFillHeight, ST77XX_DARKBLUE);
  uint64_t ID=getUniqueID();
  uint8_t Len=Format_String(Line, "#");
  Len+=Format_Hex(Line+Len, (uint16_t)(ID>>32));
  Len+=Format_Hex(Line+Len, (uint32_t)ID);
  Len+=Format_String(Line+Len, " v" VERSION);
  Line[Len]=0;
  TFT.setFont(0);
  TFT.setTextSize(1);
  TFT.setCursor(2, TFT_Footer1);
  TFT.print(Line);
  TFT.setCursor(2, TFT_Footer2);
  TFT.print("(c) Pawel Jalocha");

  TFT_DrawBatt(TFT_BattX(), TFT_BattY);
  return 1; }

int TFT_DrawSatMap(void)
{ const int16_t CenterX = TFT.width()/2-1;
  const int16_t CenterY = TFT.height()/2;
  const int16_t Radius  = TFT.height()/2; // 90deg => 120px on a 240x240 display
  const uint16_t GridColor = ST77XX_DARKGRAY;
  const uint16_t SatColor[8] = { ST77XX_WHITE,      // QZSS
                                 ST77XX_YELLOW,     // GPS
                                 ST77XX_RED,        // GLONASS
                                 ST77XX_GREEN,      // Galileo
                                 ST77XX_MAGENTA,    // BeiDou
                                 ST77XX_WHITE,
                                 ST77XX_WHITE,
                                 ST77XX_WHITE };

  TFT.fillScreen(ST77XX_DARKBLUE);

  // Elevation circles: 0deg at horizon, 90deg at zenith.
  for(uint8_t Elev=0; Elev<=90; Elev+=30)
  { int16_t Ring = ((90-Elev)*Radius + 45)/90;      // [pix]
    if(Ring>0) TFT.drawCircle(CenterX, CenterY, Ring, GridColor); }

  // Azimuth spokes every 30 degrees, with 0deg at the top.
  for(uint32_t Azim=0; Azim<360; Azim+=30)                  // [deg]
  { uint16_t Angle = (Azim*0x2000+22)/45;                   // [16-bit cordic]
     int16_t X = ((int32_t)Radius*Isin(Angle)+0x800)>>12;   // [pix]
     int16_t Y = ((int32_t)Radius*Icos(Angle)+0x800)>>12;   // [pix]
    TFT.drawLine(CenterX, CenterY, CenterX+X, CenterY+Y, GridColor); }

  // Zenith marker in the middle.
  TFT.drawCircle(CenterX, CenterY, 2, GridColor);

  for(uint8_t Idx=0; Idx<GPS_SatMon.Size; Idx++)
  { GPS_Sat &Sat = GPS_SatMon.Sat[Idx];
    int16_t R = 120-Sat.Elev*8;                             // [pix]
    if(Sat.Azim>60 || Sat.SNR==0) continue;
    uint16_t A = ((uint32_t)Sat.Azim*6*0x2000+22)/45;       // [16-bit cordic]
     int16_t X = ((int32_t)R*Isin(A)+0x800)>>12;            // [pix]
     int16_t Y = ((int32_t)R*Icos(A)+0x800)>>12;            // [pix]
    uint8_t SatRadius = Sat.SNR/6;                          // [pix] 30dB => 5pix circle
    if(SatRadius<2) SatRadius=2;
    if(Sat.Fix) TFT.fillCircle(CenterX+Y, CenterY-X, SatRadius, SatColor[Sat.Sys]);
           else TFT.drawCircle(CenterX+Y, CenterY-X, SatRadius, SatColor[Sat.Sys]); }

  return 1; }

int TFT_DrawSat(const GPS_Position *GPS)
{ char Line[32];
  // TFT.fillScreen(ST77XX_DARKBLUE);
  TFT.setTextColor(ST77XX_WHITE);
  TFT_SetMainFont();

  int Vert=TFT_TopVert;

#ifdef WITH_ST7789
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
  TFT_ClearTextLine(Vert);
  TFT.setCursor(2, Vert); TFT.print(Line); Vert+=TFT_LineVertTight;
#endif
  for(uint8_t Sys=0; Sys<=4; Sys++)      // loop over constelations
  { int Len=sprintf(Line, "%s:%d:%d", GPS_Sat::SysName(Sys), GPS_SatMon.FixSats[Sys], GPS_SatMon.VisSats[Sys]);
    uint8_t SNR=GPS_SatMon.VisSNR[Sys];  // [0.25dB]
    if(SNR>0) Len+=sprintf(Line+Len, " %4.1fdB", 0.25*SNR);
    Line[Len]=0;
    TFT_ClearTextLine(Vert);
    // int Bar=SNR/2;                       //
    // TFT.fillRect(0, Vert-11, Bar, 14, ST77XX_DARKRED);
    TFT.setCursor(2, Vert); TFT.print(Line); Vert+=TFT_LineVertTight; }
#ifdef WITH_ST7789
  if(GPS_SatCnt && GPS_SatSNR) sprintf(Line, "%02d/%02dsat %4.1fdB", GPS_Satellites, GPS_SatCnt, 0.25*GPS_SatSNR);
                        else    strcpy(Line, "--/--sat --.-dB");
  TFT_ClearTextLine(Vert);
  TFT.setCursor(2, Vert); TFT.print(Line); Vert+=TFT_LineVertTight;

  if(GPS && GPS->isValid()) sprintf(Line, "DOP%4.1f/%4.1f/%4.1f", 0.1*GPS->HDOP, 0.1*GPS->VDOP, 0.1*GPS->PDOP);
                      else   strcpy(Line, "DOP--.-/--.-/--.-");
  TFT_ClearTextLine(Vert);
  TFT.setCursor(2, Vert); TFT.print(Line); Vert+=TFT_LineVertTight;

  Line[0]=0;
  TFT_ClearTextLine(Vert);
  TFT.setCursor(2, Vert); TFT.print(Line); Vert+=TFT_LineVertTight;
  TFT_ClearTextLine(Vert);
  TFT.setCursor(2, Vert); TFT.print(Line); Vert+=TFT_LineVertTight;
#endif
  TFT_DrawBatt(TFT_BattX(), TFT_BattY);
  return 1; }

int TFT_DrawLookout(void)
{ char Line[32];
  TFT.setTextColor(ST77XX_WHITE);
  TFT_SetMainFont();
  int Vert=TFT_TopVert;

  // const char *AcftTypeName[16] = { "----", "Glid", "Tow ", "Heli",
  //                                  "SkyD", "Drop", "Hang", "Para",
  //                                  "Pwrd", "Jet ", "UFO ", "Ball",
  //                                  "Zepp", "UAV ", "Car ", "Fix " } ;

  Look.Sort_Dist();
  for( uint8_t Idx=0; Idx<Look.SortSize; Idx++)
  { const LookOut_Target *Tgt = Look.Sort[Idx]; if(!Tgt->Alloc) continue;
    uint16_t Dir=Tgt->getBearing();                                                   // [cordic]
    Dir = ((uint32_t)Dir*45+0x1000)>>13;                                             // [deg]
    uint32_t Dist=Tgt->getHorDist();                                                  // [0.5m]
    int Len=sprintf(Line, "%s %03d/%3.1fkm", Parameters.AcftTypeName(Tgt->AcftType&15), Dir, 0.0005*Dist);
    // int Len=sprintf(Line, "%02X:%06X", Tgt->AddrType, Tgt->Address);
    // if(Tgt->DistMargin) Len+=sprintf(Line+Len, " %3.1fkm", 0.0005*Tgt->HorDist);
    //                else Len+=sprintf(Line+Len, " %3.1fs", 0.5*Tgt->TimeMargin);
    TFT_ClearTextLine(Vert);
    TFT.setCursor(2, Vert); TFT.print(Line); Vert+=TFT_LineVertTight;
    if(Vert-TFT_LineVertTight>TFT.height()) break; }

  if(Vert-TFT_LineVertTight<TFT.height())
    TFT.fillRect(0, Vert-TFT_LineFillOfs, TFT.width(), TFT.height()+TFT_LineVertTight-Vert, ST77XX_DARKBLUE);
  return 1; }

int TFT_DrawRFcounts(void)
{ char Line[32];
  TFT.setTextColor(ST77XX_WHITE);
  TFT_SetMainFont();
  int Vert=TFT_TopVert;

  sprintf(Line, "FLR: %d", Radio_RxCount[0]);
  TFT_ClearTextLine(Vert);
  TFT.setCursor(2, Vert); TFT.print(Line); Vert+=TFT_LineVertTight;

  sprintf(Line, "OGN: %d", Radio_RxCount[1]);
  TFT_ClearTextLine(Vert);
  TFT.setCursor(2, Vert); TFT.print(Line); Vert+=TFT_LineVertTight;

  sprintf(Line, "MDR: %d", Radio_RxCount[2]);
  TFT_ClearTextLine(Vert);
  TFT.setCursor(2, Vert); TFT.print(Line); Vert+=TFT_LineVertTight;

  sprintf(Line, "LDR: %d", Radio_RxCount[5]);
  TFT_ClearTextLine(Vert);
  TFT.setCursor(2, Vert); TFT.print(Line); Vert+=TFT_LineVertTight;

  sprintf(Line, "HDR: %d", Radio_RxCount[6]);
  TFT_ClearTextLine(Vert);
  TFT.setCursor(2, Vert); TFT.print(Line); Vert+=TFT_LineVertTight;

  // sprintf(Line, "FNT: %d", Radio_RxCount[4]);
  // TFT_ClearTextLine(Vert);
  // TFT.setCursor(2, Vert); TFT.print(Line); Vert+=TFT_LineVertTight;

  TFT_ClearTextLine(Vert);
  return 1; }

int TFT_DrawRF(void)
{ char Line[32];
  TFT.setTextColor(ST77XX_WHITE);
  TFT_SetMainFont();

  int Vert=TFT_TopVert;
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
  TFT_ClearTextLine(Vert);
  TFT.setCursor(2, Vert); TFT.print(Line); Vert+=TFT_LineVert;

  sprintf(Line, "Rx: %+4.1fdBm", Radio_BkgRSSI);
  TFT_ClearTextLine(Vert);
  TFT.setCursor(2, Vert); TFT.print(Line); Vert+=TFT_LineVert;
  uint32_t Sum=0;
  for(uint8_t Sys=0; Sys<8; Sys++)
    Sum+=Radio_RxCount[Sys];
  sprintf(Line, "Rx: %d pkt", Sum);
  TFT_ClearTextLine(Vert);
  TFT.setCursor(2, Vert); TFT.print(Line); Vert+=TFT_LineVert;

  Len=0;
  Len+=Format_String(Line+Len, Radio_FreqPlan.getPlanName());               // name of the frequency plan
  Line[Len++]=' ';
  Len+=Format_UnsDec(Line+Len, (uint32_t)(Radio_FreqPlan.getCenterFreq()/100000), 3, 1); // center frequency
  Len+=Format_String(Line+Len, "M");
  Line[Len]=0;
  TFT_ClearTextLine(Vert);
  TFT.setCursor(2, Vert); TFT.print(Line); Vert+=TFT_LineVert;

  TFT_ClearTextLine(Vert); Vert+=TFT_LineVert;

#ifdef WITH_ST7789
  sprintf(Line, "FLR: %d", Radio_RxCount[0]);
  TFT_ClearTextLine(Vert);
  TFT.setCursor(2, Vert); TFT.print(Line); Vert+=TFT_LineVertTight;

  sprintf(Line, "OGN: %d", Radio_RxCount[1]);
  TFT_ClearTextLine(Vert);
  TFT.setCursor(2, Vert); TFT.print(Line); Vert+=TFT_LineVertTight;

  sprintf(Line, "MDR: %d", Radio_RxCount[2]);
  TFT_ClearTextLine(Vert);
  TFT.setCursor(2, Vert); TFT.print(Line); Vert+=TFT_LineVertTight;

  sprintf(Line, "LDR: %d", Radio_RxCount[5]);
  TFT_ClearTextLine(Vert);
  TFT.setCursor(2, Vert); TFT.print(Line); Vert+=TFT_LineVertTight;

  sprintf(Line, "HDR: %d", Radio_RxCount[6]);
  TFT_ClearTextLine(Vert);
  TFT.setCursor(2, Vert); TFT.print(Line); Vert+=TFT_LineVertTight;
#endif

  return 1; }

int TFT_DrawBaro(const GPS_Position *GPS)
{ char Line[32];
  // TFT.fillScreen(ST77XX_DARKBLUE);
  TFT.setTextColor(ST77XX_WHITE);
  TFT_SetMainFont();
  if(!GPS || !GPS->hasBaro) return 0;

  int Vert=TFT_TopVert;
  uint8_t Len=Format_String(Line+Len, "Baro ");
  if(GPS && GPS->hasBaro)
  { Len+=Format_UnsDec(Line+Len, (GPS->Pressure+20)/40, 5, 1);
    Len+=Format_String(Line+Len, "hPa "); }
  else Len+=Format_String(Line+Len, "----.-hPa ");
  Line[Len]=0;
  TFT_ClearTextLine(Vert);
  TFT.setCursor(2, Vert); TFT.print(Line); Vert+=TFT_LineVert;

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
  TFT_ClearTextLine(Vert);
  TFT.setCursor(2, Vert); TFT.print(Line); Vert+=TFT_LineVert;

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
  TFT_ClearTextLine(Vert);
  TFT.setCursor(2, Vert); TFT.print(Line); Vert+=TFT_LineVert;

  TFT_ClearTextLine(Vert);
  if(GPS && GPS->hasBaro)
  { float Dew = DewPoint(0.1f*GPS->Temperature, 0.1f*GPS->Humidity);
    sprintf(Line, "Dew: %+5.1fC", Dew);
    Line[5]=0xB0;
    TFT.setCursor(2, Vert); TFT.print(Line);
  }

  Vert+=TFT_LineVert;
  TFT_ClearTextLine(Vert);
  return 1; }

static void ClearLine(int Vert, int Height=TFT_LineFillHeight, int VertOfs=TFT_LineFillOfs)
{ TFT.fillRect(0, Vert-VertOfs, TFT.width(), Height, ST77XX_DARKBLUE); }

#ifdef WITH_SPIFFS
int TFT_DrawLog(const GPS_Position *GPS)
{ char Line[32];
  TFT.setTextColor(ST77XX_WHITE);
  TFT_SetMainFont();
  int Vert=TFT_TopVert;

  size_t Total, Used;
  int Err=SPIFFS_Info(Total, Used);
  if(Err==0)
  { sprintf(Line, "Used:%3.1fMB", (float)Used/0x100000);
    ClearLine(Vert); TFT.setCursor(2, Vert); TFT.print(Line); Vert+=TFT_LineVertTight;
    sprintf(Line, "Free:%3.1fMB", (float)(Total-Used)/0x100000);
    ClearLine(Vert); TFT.setCursor(2, Vert); TFT.print(Line); Vert+=TFT_LineVertTight;
    Line[0]=0;
    // if(FlashLog_FileName[0]) sprintf(Line, "%s", FlashLog_FileName+strlen(FlashLog_Path)+1);
    if(FlashLog_FileTime) { sprintf(Line, "File: "); Format_HHMMSS(Line+6, FlashLog_FileTime); Line[6+6]=0; }
    ClearLine(Vert); TFT.setCursor(2, Vert); TFT.print(Line); Vert+=TFT_LineVertTight;
    Line[0]=0;
    if(FlashLog_FileFlush) sprintf(Line, "Size:%5dkB", (FlashLog_FileFlush+512)>>10);
    ClearLine(Vert); TFT.setCursor(2, Vert); TFT.print(Line); Vert+=TFT_LineVertTight;
    Line[0]=0;
    if(FlashLog_Files>0) sprintf(Line, "%d log files", FlashLog_Files);
    ClearLine(Vert); TFT.setCursor(2, Vert); TFT.print(Line); Vert+=TFT_LineVertTight; }
  else
  { sprintf(Line, "Log:<info-error>");
    ClearLine(Vert, TFT_LineFillHeight*5); TFT.setCursor(2, Vert); TFT.print(Line); Vert+=TFT_LineVertTight; }
  return 1; }
#else
int TFT_DrawLog(const GPS_Position *GPS) { return 0; }
#endif

#ifdef WITH_LORAWAN
int TFT_DrawLoRaWAN(const GPS_Position *GPS)
{ char Line[32];
  TFT.setTextColor(ST77XX_WHITE);
  TFT_SetMainFont();

  int Vert=TFT_TopVert;
  const char *StateName[4] = { "Not-conn.", "Join-Req", "+Joined+", "Pkt-Sent" } ;
  int Len=Format_String(Line, "TTN: ");
  if(WANdev.Enable)
  { if(WANdev.State==2) Len+=Format_Hex(Line+Len, WANdev.DevAddr);
    else if(WANdev.State<=3) Len+=Format_String(Line+Len, StateName[WANdev.State]);
                        else Len+=Format_Hex(Line+Len, WANdev.State); }
  else
  { Len+=Format_String(Line+Len, "Disabled"); }
  Line[Len]=0;
  TFT_ClearTextLine(Vert);
  TFT.setCursor(2, Vert); TFT.print(Line); Vert+=TFT_LineVert;

  if(WANdev.State>=2)
  { Len=0; // Len =Format_String(Line    , "^^");
    Len+=Format_Hex(Line+Len, (uint16_t)WANdev.UpCount);
    Len+=Format_String(Line+Len, " >> ");
    Len+=Format_Hex(Line+Len, (uint16_t)WANdev.DnCount);
    Line[Len]=0;
    TFT_ClearTextLine(Vert);
    TFT.setCursor(2, Vert); TFT.print(Line); Vert+=TFT_LineVert;

    // Len =Format_String(Line    , "Rx:");
    Len=0;
    Len+=Format_SignDec(Line+Len, ((int32_t)WANdev.RxSNR*10+2)>>2, 2, 1);
    Len+=Format_String(Line+Len, "dB ");
    Len+=Format_SignDec(Line+Len, (int32_t)WANdev.RxRSSI, 3);
    Len+=Format_String(Line+Len, "dBm");
    Line[Len]=0;
    TFT_ClearTextLine(Vert);
    TFT.setCursor(2, Vert); TFT.print(Line); Vert+=TFT_LineVert; }
  else
  { TFT_ClearTextLine(Vert); Vert+=TFT_LineVert;
    TFT_ClearTextLine(Vert); Vert+=TFT_LineVert; }

  TFT_ClearTextLine(Vert);
  Len=Format_String(Line, "Key: ");
  Len+=Format_HexBytes(Line+Len, WANdev.AppKey, 2); Line[Len++]='.'; Line[Len++]='.';
  Len+=Format_Hex(Line+Len, WANdev.AppKey[15]); Line[Len]=0;
  TFT.setCursor(2, Vert); TFT.print(Line); Vert+=TFT_LineVert;
  TFT_ClearTextLine(Vert); Vert+=TFT_LineVert;

  return 1; }
#else
int TFT_DrawLoRaWAN(const GPS_Position *GPS) { return 0; }
#endif


int TFT_DrawGPS(const GPS_Position *GPS)
{ char Line[32];
  // TFT.fillScreen(ST77XX_DARKBLUE);
  TFT.setTextColor(ST77XX_WHITE);
  TFT_SetMainFont();

  int Vert=TFT_TopVert;

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
  TFT_ClearTextLine(Vert);
  TFT.setCursor(2, Vert); TFT.print(Line); Vert+=TFT_LineVert;

  Len=0;
  Line[Len++]=' ';
  if(GPS && GPS->isValid()) Len+=Format_SignDec(Line+Len,  GPS->Latitude /6, 7, 5);
                      else  Len+=Format_String(Line+Len, "---.-----");
  Line[Len]=0;
  TFT_ClearTextLine(Vert);
  TFT.setCursor(2, Vert); TFT.print(Line);
  TFT.setCursor(TFT.getCursorX(), Vert-4); TFT.write('o');
  Vert+=TFT_LineVert;

  Len=0;
  if(GPS && GPS->isValid()) Len+=Format_SignDec(Line+Len,  GPS->Longitude /6, 8, 5);
                      else  Len+=Format_String(Line+Len, "----.-----");
  Line[Len]=0;
  TFT_ClearTextLine(Vert);
  TFT.setCursor(2, Vert); TFT.print(Line);
  TFT.setCursor(TFT.getCursorX(), Vert-4); TFT.write('o');
  Vert+=TFT_LineVert;

  if(GPS && GPS->isValid()) sprintf(Line, "%7.1fm MSL", 0.1*GPS->Altitude);
                      else   strcpy(Line, "-----.-m MSL");
  TFT_ClearTextLine(Vert);
  TFT.setCursor(2, Vert); TFT.print(Line); Vert+=TFT_LineVert;

#ifdef WITH_ST7789
  if(GPS && GPS->isValid()) sprintf(Line, "%03d/%5.1fm/s", (GPS->Heading+5)/10, 0.1*GPS->Speed);
                      else   strcpy(Line, "---/  -.-m/s");
  TFT_ClearTextLine(Vert);
  TFT.setCursor(2, Vert); TFT.print(Line); Vert+=TFT_LineVert;

  if(GPS && GPS->isValid()) sprintf(Line, "%+5.1fm/s", 0.1*GPS->ClimbRate);
                       else  strcpy(Line, " --.-m/s");
  TFT_ClearTextLine(Vert);
  TFT.setCursor(2, Vert); TFT.print(Line); Vert+=TFT_LineVert;

  if(GPS_SatCnt && GPS_SatSNR) sprintf(Line, "%02d/%02dsat %4.1fdB", GPS_Satellites, GPS_SatCnt, 0.25*GPS_SatSNR);
                        else    strcpy(Line, "--/--sat --.-dB");
  TFT_ClearTextLine(Vert);
  TFT.setCursor(2, Vert); TFT.print(Line); Vert+=TFT_LineVert;

  if(GPS && GPS->isValid()) sprintf(Line, "DOP%4.1f/%4.1f/%4.1f", 0.1*GPS->HDOP, 0.1*GPS->VDOP, 0.1*GPS->PDOP);
                      else   strcpy(Line, "DOP--.-/--.-/--.-");
  TFT_ClearTextLine(Vert);
  TFT.setCursor(2, Vert); TFT.print(Line); Vert+=TFT_LineVert;

  if(GPS && GPS->isValid()) sprintf(Line, "Geoid%+6.1fm", 0.1*GPS->GeoidSeparation);
                      else   strcpy(Line, "Geoid  --.-m");
  TFT_ClearTextLine(Vert);
  TFT.setCursor(2, Vert); TFT.print(Line); Vert+=TFT_LineVert;
#endif

  TFT_ClearTextLine(Vert);
  TFT_ClearTextLine(Vert+TFT_LineVert);
  TFT_DrawBatt(TFT_BattX(), TFT_BattY);
  return 1; }

#endif // WITH_ST7735 || WITH_ST7789 || WITH_ILI9341
