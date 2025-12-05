
#include "epd.h"

#include "proc.h"
#include "gps.h"

#ifdef WITH_EPAPER

// ========================================================================================================================

SPIClass hSPI(HSPI);                         // SPI port for the e-paper (not to conflict with Radio SPI

GxEPD2_BW<GxEPD2_154_D67, GxEPD2_154_D67::HEIGHT> EPD(GxEPD2_154_D67(EPD_PinCS, EPD_PinDC, EPD_PinRST, EPD_PinBUSY));

void EPD_Init(void)                         // start the e-paper display
{ hSPI.begin(EPD_PinSCK, EPD_PinMISO, EPD_PinMOSI, EPD_PinCS);              // separate SPI not to conflict with Radio
  EPD.epd2.selectSPI(hSPI, SPISettings(4000000, MSBFIRST, SPI_MODE0));
  EPD.init(115200, true, 10, true);                                         // 10 ms reset
  EPD.setRotation(0); }

// ========================================================================================================================

static void greyRect(int16_t X, int16_t Y, int16_t W, int16_t H, int16_t Step=3)  // 50% grey rectangle by setting every 2nd pixel
{ int16_t Odd=0;
  for(int16_t y=Y; y<=Y+H; y++)
  { for(int16_t x=X+Odd; x<=X+W; x+=Step)                                         // or maybe every 3rd pixel ?
      EPD.drawPixel(x, y, GxEPD_BLACK);
    Odd++; if(Odd>=Step) Odd=0; }
}

static uint32_t Hash(const uint32_t *Data, int Size)
{ uint32_t Sum=0;
  for(int Idx=0; Idx<Size; Idx++)
    Sum+=Data[Idx];
  return Sum; }

// ========================================================================================================================

static const uint8_t MaxSats=32;
static const uint8_t MaxDrawSats=16; // max. number of SNR bars to display

static GPS_Sat SatList[MaxSats];

static uint8_t qSec=15;
static uint8_t DrawSats=0;
static uint32_t SatHash=0;

static uint8_t SortSatMon(void)
{ uint8_t Size=0;
  for(uint8_t Idx=0; Idx<GPS_SatMon.Size; Idx++)
  { if(Size>=MaxSats) break;
    GPS_Sat &Sat = GPS_SatMon.Sat[Idx]; if(Sat.SNR==0) continue;
    SatList[Size++]=Sat; }
  if(Size>1) std::sort(SatList, SatList+Size, GPS_SatList::HigherSNR);
  if(Size>MaxDrawSats) Size=MaxDrawSats;
  return Size; }

static void DrawSatMon(uint8_t Size)
{ uint8_t Pos=0;
  for(uint8_t Idx=0; Idx<Size; Idx++)
  { GPS_Sat &Sat = SatList[Size-Idx-1];
    uint8_t H = Sat.SNR; if(H>40) H=40;
    if(Sat.Fix) greyRect(Pos, 40-H, 4, H, 2);
    EPD.drawRect(Pos, 40-H, 4, H, GxEPD_BLACK);
    Pos+=4; }
}

static bool UpdateSatMon(void)
{ if(qSec==GPS_SatMon.qSec) return 0;
  qSec=GPS_SatMon.qSec;
  uint8_t Size=SortSatMon();
  uint32_t NewHash=Hash((const uint32_t *)SatList, Size);
  if(NewHash==SatHash) return 0;
  SatHash=NewHash;
  uint8_t ClearSize=Size; if(DrawSats>ClearSize) ClearSize=DrawSats;
  DrawSats=Size;
  if(ClearSize==0) return 0;
  EPD.setPartialWindow(0, 0, ClearSize*4, 40);                       // partial update
  EPD.fillRect(0, 0, ClearSize*4, 40, GxEPD_WHITE);                  // clear the area to be redrawn
  EPD.firstPage();
  DrawSatMon(Size);
  EPD.nextPage();
  return 1; }

// ========================================================================================================================

// static uint8_t AlarmLevel = 4;   // now in proc.h/.cpp
static uint8_t PrevAlarmLevel = 0;

static void DrawAlarmFrame(int16_t X, int16_t Y, int16_t W, uint8_t Color=GxEPD_BLACK)
{ int16_t W2 = W/2;
  EPD.drawLine(X, Y, X+W2, Y+W, Color);
  EPD.drawLine(X, Y, X-W2, Y+W, Color);
  EPD.drawLine(X-W2, Y+W, X+W2, Y+W, Color); }

static void DrawAlarmLevel(void)
{ DrawAlarmFrame(110, 0, 34);
  EPD.setTextColor(GxEPD_BLACK);
  EPD.setFont(&FreeMonoBold12pt7b);
  EPD.drawChar(110-6, 28, '0'+AlarmLevel, GxEPD_BLACK, GxEPD_WHITE, 1);
  PrevAlarmLevel=AlarmLevel; }

static bool UpdateAlarmLevel(void)
{ if(PrevAlarmLevel==AlarmLevel) return 0;
  // PrevAlarmLevel=AlarmLevel;
  EPD.setPartialWindow(110-17, 0, 35, 35);                       // partial update
  EPD.fillRect(110-17, 0, 35, 35, GxEPD_WHITE);                  // clear the area to be redrawn
  EPD.firstPage();
  DrawAlarmLevel();
  EPD.nextPage();
  return 1; }

// ========================================================================================================================

static void drawSpeaker(int16_t X, int16_t Y, int16_t W, uint8_t Color=GxEPD_BLACK)
{ int16_t W2 = W/2;
  int16_t W4 = W/4;
  EPD.drawRect(X-W4, Y-W4, W2+1, W2+1, Color);
  EPD.drawLine(X+W4, Y+W4, X+W2, Y+W2, Color);
  EPD.drawLine(X+W4, Y-W4, X+W2, Y-W2, Color);
  EPD.drawLine(X+W2, Y-W2, X+W2, Y+W2, Color); }

// ========================================================================================================================

static uint8_t  PrevBattLev = 0;

static void DrawBattFrame(void)
{ EPD.drawRect(149, 0, 50, 21, GxEPD_BLACK);                     // draw empty battery symbol
  EPD.fillRect(145, 5,  5, 10, GxEPD_BLACK);
  PrevBattLev=0; }

static bool UpdateBatt(void)
{ char Line[16];

  int16_t BattVolt=(BatteryVoltage+128)>>8;                      // [mV] measured and averaged  battery voltage
  int16_t BattLev=(BattVolt-3300)/8;
  if(BattLev<0) BattLev=0;
  else if(BattLev>100) BattLev=100;
  if(BattLev==PrevBattLev) return 0;

  EPD.setPartialWindow(145, 0, 55, 21);                          // partial update: the inside of the battery box
  EPD.firstPage();
  EPD.fillRect(140, 0, 55, 21, GxEPD_WHITE);                     // clear the area to be redrawn
  if(BattLev>2) greyRect(199-BattLev/2, 1, BattLev/2, 19, 3);
  EPD.setTextColor(GxEPD_BLACK);
  EPD.setFont(&FreeMonoBold9pt7b);                               // use bold font: more readable
  EPD.setCursor(154, 15);
  sprintf(Line, "%3d%%", BattLev);
  EPD.print(Line);
  DrawBattFrame();
  EPD.nextPage();
  PrevBattLev=BattLev;
  return 1; }

// ========================================================================================================================

#include "OGN_Logo_200x200.xbm"              // OGN logo: bits in bytes are of wrong order thus need to be reversed

static uint8_t ReverseBits(uint8_t X)        // reverse bytes in the OGN logo, apparently they have wrong bit order
{ X = ( X      >>4) | ( X      <<4);
  X = ((X&0xCC)>>2) | ((X&0x33)<<2);
  X = ((X&0xAA)>>1) | ((X&0x55)<<1);
  return X; }

static void DrawLogo(void)
{ int Size=OGN_Logo_200x200_width*OGN_Logo_200x200_height/8;   // [bytes] size of the OGN Logo file
  uint8_t *BitMap = (uint8_t *)malloc(Size);                   // allocate bitmap with reversed bytes
  for(int Idx=0; Idx<Size; Idx++)                              // there seemed to be no other way
  { BitMap[Idx]=ReverseBits(OGN_Logo_200x200[Idx]); }          // but to manually reverse bits in every byte
  EPD.drawBitmap(0, 0, BitMap, OGN_Logo_200x200_width, OGN_Logo_200x200_height, GxEPD_BLACK);
  free(BitMap); }                                              // free the allocated bitmap

// ========================================================================================================================

static uint32_t UpdateTime = 0;
static uint32_t RedrawTime = 0;

void EPD_DrawID(void)
{ char Line[32];
  EPD.setFullWindow();                                           // this will be full page update
  EPD.firstPage();
  EPD.fillScreen(GxEPD_WHITE);                                   // all-white screen
  DrawLogo();
  EPD.setTextColor(GxEPD_BLACK);
  EPD.setFont(&FreeMonoBold9pt7b);                               // use bold font: more readable
  sprintf(Line, "%X:%d:%06X %s", Parameters.AcftType, Parameters.AddrType, Parameters.Address, Parameters.Reg);
  EPD.setCursor(0, 195);
  EPD.print(Line);
  // drawSpeaker(110, 16, 32, GxEPD_BLACK);
  DrawAlarmLevel();
  DrawBattFrame();
  EPD.nextPage();                                                // put full page onto the e-paper (takes 2 sec)
  UpdateTime = millis();
  RedrawTime=UpdateTime; }

void EPD_UpdateID(void)
{ uint32_t msTime=millis();
  uint32_t msAge = msTime-RedrawTime;
  if(msAge>=120000) EPD_DrawID();                                // redraw every 10 minutes
  else
  { msAge = msTime-UpdateTime;
    if(msAge<1000) return; }                                     // do not update more frequent than once per 2 seconds
  UpdateAlarmLevel();
  UpdateBatt();
  UpdateSatMon();
  UpdateTime=msTime; }

// ========================================================================================================================

void EPD_Task(void *Parms)
{
  EPD_Init();
  EPD_DrawID();

  for( ; ; )
  { vTaskDelay(100);
    EPD_UpdateID();                  // this can take seconds (occasionally)
  }
}

// ========================================================================================================================

#endif
