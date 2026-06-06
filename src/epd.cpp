
#include "epd.h"

#include "proc.h"
#include "gps.h"
#include "intmath.h"

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

static uint8_t PrevAlarmThresh = 0;
static const int16_t AlarmX = 124;
static const int16_t AlarmY = 0;
static const int16_t AlarmW = 34;

static void DrawBellFrame(int16_t X, int16_t Y, int16_t W, uint8_t Color=GxEPD_BLACK)
{ int16_t W2 = W/2;
  int16_t L = X-W2+4;
  int16_t R = X+W2-4;
  int16_t Top = Y+4;
  int16_t BellTop = Y+8;
  int16_t Bottom = Y+W-5;
  EPD.drawLine(X-3, Top, X+3, Top, Color);                  // hanger
  EPD.drawLine(X-8, BellTop+4, X-8, Bottom-4, Color);       // left side
  EPD.drawLine(X+8, BellTop+4, X+8, Bottom-4, Color);       // right side
  EPD.drawLine(X-8, BellTop+4, X-5, BellTop, Color);        // upper left curve
  EPD.drawLine(X+8, BellTop+4, X+5, BellTop, Color);        // upper right curve
  EPD.drawLine(X-5, BellTop, X+5, BellTop, Color);          // top arch
  EPD.drawLine(L, Bottom, R, Bottom, Color);                // lower rim
  EPD.drawLine(L, Bottom, X-W2, Bottom+4, Color);
  EPD.drawLine(R, Bottom, X+W2, Bottom+4, Color);
  EPD.drawLine(X-W2, Bottom+4, X+W2, Bottom+4, Color);
  EPD.fillCircle(X, Bottom+6, 2, Color); }                  // clapper

static void DrawAlarmThresh(void)
{ DrawBellFrame(AlarmX, AlarmY, AlarmW);
  if(AlarmThresh>=4)
  { EPD.drawLine(AlarmX-6, 15, AlarmX+6, 27, GxEPD_BLACK);
    EPD.drawLine(AlarmX-5, 15, AlarmX+7, 27, GxEPD_BLACK);
    EPD.drawLine(AlarmX+6, 15, AlarmX-6, 27, GxEPD_BLACK);
    EPD.drawLine(AlarmX+5, 15, AlarmX-7, 27, GxEPD_BLACK); }
  else if(AlarmThresh>0)
  { EPD.setTextColor(GxEPD_BLACK);
    EPD.setFont(&FreeMonoBold9pt7b);
    EPD.drawChar(AlarmX-5, 25, '0'+AlarmThresh, GxEPD_BLACK, GxEPD_WHITE, 1); }
  PrevAlarmThresh=AlarmThresh; }

static bool UpdateAlarmThresh(void)
{ if(PrevAlarmThresh==AlarmThresh) return 0;
  // PrevAlarmThresh=AlarmThresh;
  EPD.setPartialWindow(AlarmX-17, AlarmY, 35, 39);               // partial update
  EPD.fillRect(AlarmX-17, AlarmY, 35, 39, GxEPD_WHITE);          // clear the area to be redrawn
  EPD.firstPage();
  DrawAlarmThresh();
  EPD.nextPage();
  return 1; }

// ========================================================================================================================

static uint8_t PrevAcftCount = 0xFF;

static uint8_t getAcftCount(void)
{
#ifdef WITH_LOOKOUT
  return Look.Targets;
#else
  return 0;
#endif
}

static void DrawAcftIcon(int16_t X, int16_t Y, uint8_t Color=GxEPD_BLACK)
{ EPD.drawLine(X-11, Y,   X+10, Y,   Color); // fuselage
  EPD.drawLine(X,    Y-7, X,    Y+7, Color); // wings
  EPD.drawLine(X-8,  Y-3, X-8,  Y+3, Color); // tail
  EPD.drawPixel(X+11, Y, Color); }

static void DrawAcftCount(void)
{ char Line[8];
  uint8_t Count = getAcftCount();
  if(Count>99) Count=99;
  DrawAcftIcon(86, 9);
  EPD.setTextColor(GxEPD_BLACK);
  EPD.setFont(&FreeMonoBold12pt7b);
  EPD.setCursor(69, 35);
  sprintf(Line, "%2u", Count);
  EPD.print(Line);
  PrevAcftCount=Count; }

static bool UpdateAcftCount(void)
{ uint8_t Count = getAcftCount();
  if(Count>99) Count=99;
  if(PrevAcftCount==Count) return 0;
  EPD.setPartialWindow(68, 0, 38, 38);                         // partial update
  EPD.fillRect(68, 0, 38, 38, GxEPD_WHITE);                    // clear the area to be redrawn
  EPD.firstPage();
  DrawAcftCount();
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

static void DrawLogoCopyright(void)
{ const char *Text = "v" VERSION " (c) Pawel Jalocha";
  EPD.fillRect(188, 68, 12, 112, GxEPD_WHITE);
  EPD.setTextColor(GxEPD_BLACK);
  EPD.setFont(0);
  EPD.setTextSize(1);
  EPD.setRotation(3);
  EPD.setCursor(28, 193);
  EPD.print(Text);
  EPD.setRotation(0); }

// ========================================================================================================================

static const int16_t TrafficMapX = 0;
static const int16_t TrafficMapY = 42;
static const int16_t TrafficMapW = 200;
static const int16_t TrafficMapH = 140;
static const int16_t TrafficMapCenterX = 100;
static const int16_t TrafficMapCenterY = 112;
static const int16_t TrafficMapRadius  = 68;
static const int16_t TrafficMapRange[] = { 250, 500, 1000, 2000, 4000, 8000, 16000 }; // [m] map range: outer circle
static uint8_t TrafficMapRangeIdx = 4;

static uint32_t TrafficMapHash = 0;
static uint32_t TrafficMapTime = 0;
static uint8_t  TrafficMapWarn = 0;
static bool PrevGPSLock = false;

static bool hasStableGPSLock(void)
{ return GPS_TimeSinceLock>10; }

static uint16_t TrafficMapHeading(void)
{
#ifdef WITH_LOOKOUT
  if(Look.Pos.Speed>2) return Look.Pos.Heading;                  // [0.5m/s] use track-up above 1m/s
#endif
  return 0;                                                      // otherwise north-up
}

static void DrawTrafficGrid(void)
{ char Line[8];
  uint16_t Heading = TrafficMapHeading();
  uint16_t Degrees = ((uint32_t)Heading*45+0x1000)>>13;
  if(Degrees>=360) Degrees-=360;
  EPD.drawCircle(TrafficMapCenterX, TrafficMapCenterY, TrafficMapRadius, GxEPD_BLACK);
  EPD.drawCircle(TrafficMapCenterX, TrafficMapCenterY, TrafficMapRadius/2, GxEPD_BLACK);
  EPD.drawCircle(TrafficMapCenterX, TrafficMapCenterY, TrafficMapRadius/4, GxEPD_BLACK);
  EPD.drawLine(TrafficMapCenterX, TrafficMapCenterY-TrafficMapRadius, TrafficMapCenterX, TrafficMapCenterY+TrafficMapRadius, GxEPD_BLACK);
  EPD.drawLine(TrafficMapCenterX-TrafficMapRadius, TrafficMapCenterY, TrafficMapCenterX+TrafficMapRadius, TrafficMapCenterY, GxEPD_BLACK);
  EPD.drawLine(TrafficMapCenterX-3, TrafficMapCenterY, TrafficMapCenterX+3, TrafficMapCenterY, GxEPD_BLACK);
  EPD.drawLine(TrafficMapCenterX, TrafficMapCenterY-3, TrafficMapCenterX, TrafficMapCenterY+3, GxEPD_BLACK);
  EPD.setTextColor(GxEPD_BLACK);
  EPD.setFont(&FreeMonoBold9pt7b);
  sprintf(Line, "%03u", Degrees);
  EPD.setCursor(TrafficMapCenterX-18, TrafficMapCenterY-TrafficMapRadius+14);
  EPD.print(Line);
  if(TrafficMapRange[TrafficMapRangeIdx]<1000) sprintf(Line, "%dm", TrafficMapRange[TrafficMapRangeIdx]);
                                          else sprintf(Line, "%dkm", TrafficMapRange[TrafficMapRangeIdx]/1000);
  EPD.setFont(&FreeMonoBold9pt7b);
  EPD.setCursor(158, 179);
  EPD.print(Line); }

static void DrawTrafficTarget(int16_t X, int16_t Y, uint16_t Heading, uint8_t Warn)
{ const int16_t Nose = 8;
  const int16_t Tail = 5;
  const int16_t Wing = 6;
  int16_t Sin = Isin(Heading);
  int16_t Cos = Icos(Heading);
  int16_t NoseX = X + ((int32_t)Nose*Sin+0x800)/0x1000;
  int16_t NoseY = Y - ((int32_t)Nose*Cos+0x800)/0x1000;
  int16_t TailX = X - ((int32_t)Tail*Sin+0x800)/0x1000;
  int16_t TailY = Y + ((int32_t)Tail*Cos+0x800)/0x1000;
  int16_t WingX = ((int32_t)Wing*Cos+0x800)/0x1000;
  int16_t WingY = ((int32_t)Wing*Sin+0x800)/0x1000;
  EPD.drawLine(TailX, TailY, NoseX, NoseY, GxEPD_BLACK);
  EPD.drawLine(X-WingX, Y-WingY, X+WingX, Y+WingY, GxEPD_BLACK);
  if(Warn) EPD.drawCircle(X, Y, 4+Warn, GxEPD_BLACK); }

static void DrawThickLine(int16_t X1, int16_t Y1, int16_t X2, int16_t Y2)
{ EPD.drawLine(X1,   Y1,   X2,   Y2,   GxEPD_BLACK);
  EPD.drawLine(X1-1, Y1,   X2-1, Y2,   GxEPD_BLACK);
  EPD.drawLine(X1+1, Y1,   X2+1, Y2,   GxEPD_BLACK);
  EPD.drawLine(X1,   Y1-1, X2,   Y2-1, GxEPD_BLACK);
  EPD.drawLine(X1,   Y1+1, X2,   Y2+1, GxEPD_BLACK); }

static void DrawTrafficAlert(uint16_t MapHeading)
{
#ifdef WITH_LOOKOUT
  if(Look.WarnLevel==0) return;
  if(Look.WorstTgtIdx>=Look.MaxTargets) return;
  const LookOut_Target *Tgt = Look.Target+Look.WorstTgtIdx;
  if(!Tgt->Alloc) return;

  int32_t dX = (int32_t)Tgt->Pos.X - Look.Pos.X;
  int32_t dY = (int32_t)Tgt->Pos.Y - Look.Pos.Y;
  int32_t Dist = Acft_RelPos::FastDistance((int16_t)dX, (int16_t)dY);
  if(Dist<=0) return;

  int16_t Sin = Isin(MapHeading);
  int16_t Cos = Icos(MapHeading);
  int32_t Fwd = (dX*Cos + dY*Sin + 0x800)>>12;
  int32_t Right = (dY*Cos - dX*Sin + 0x800)>>12;

  const int16_t ArrowRadius = TrafficMapRadius-9;
  int16_t TipX = TrafficMapCenterX + (Right*ArrowRadius)/Dist;
  int16_t TipY = TrafficMapCenterY - (Fwd*ArrowRadius)/Dist;
  int16_t VecX = TipX-TrafficMapCenterX;
  int16_t VecY = TipY-TrafficMapCenterY;
  int16_t Len = Acft_RelPos::FastDistance(VecX, VecY);
  if(Len<=0) return;

  int16_t BaseX = TipX - ((int32_t)VecX*15)/Len;
  int16_t BaseY = TipY - ((int32_t)VecY*15)/Len;
  int16_t WingX = ((int32_t)VecY*8)/Len;
  int16_t WingY = ((int32_t)VecX*8)/Len;

  DrawThickLine(TrafficMapCenterX, TrafficMapCenterY, TipX, TipY);
  DrawThickLine(TipX, TipY, BaseX+WingX, BaseY-WingY);
  DrawThickLine(TipX, TipY, BaseX-WingX, BaseY+WingY);

  char Line[12];
  EPD.fillRect(70, 120, 60, 20, GxEPD_WHITE);
  EPD.setTextColor(GxEPD_BLACK);
  EPD.setFont(&FreeMonoBold12pt7b);
  EPD.setCursor(76, 138);
  if(Tgt->TimeMargin<0xFF) sprintf(Line, "%us", (Tgt->TimeMargin+1)/2);
                      else sprintf(Line, "W%u", Look.WarnLevel);
  EPD.print(Line);
#endif
}

static uint32_t CalcTrafficMapHash(void)
{
#ifdef WITH_LOOKOUT
  uint32_t Hash = Look.Targets + ((uint32_t)TrafficMapRangeIdx<<24) + ((uint32_t)TrafficMapHeading()<<8);
  Hash += ((uint32_t)Look.WarnLevel<<29) + ((uint32_t)Look.WorstTgtIdx<<20);
  for(uint8_t Idx=0; Idx<Look.MaxTargets; Idx++)
  { const LookOut_Target *Tgt = Look.Target+Idx; if(!Tgt->Alloc) continue;
    int32_t dX = (int32_t)Tgt->Pos.X - Look.Pos.X;
    int32_t dY = (int32_t)Tgt->Pos.Y - Look.Pos.Y;
    Hash += Tgt->ID;
    Hash += ((uint32_t)(dX>>5)&0x7FF)<< 0;
    Hash += ((uint32_t)(dY>>5)&0x7FF)<<11;
    Hash += ((uint32_t)(Tgt->Pos.Heading>>8)&0xFF)<<19;
    Hash += ((uint32_t)Tgt->WarnLevel)<<28; }
  return Hash;
#else
  return 0;
#endif
}

static void DrawTrafficMap(void)
{ DrawTrafficGrid();
#ifdef WITH_LOOKOUT
  const int32_t MaxDist = (int32_t)TrafficMapRange[TrafficMapRangeIdx]*2; // [0.5m]
  uint16_t Heading = TrafficMapHeading();
  int16_t Sin = Isin(Heading);
  int16_t Cos = Icos(Heading);
  for(uint8_t Idx=0; Idx<Look.MaxTargets; Idx++)
  { const LookOut_Target *Tgt = Look.Target+Idx; if(!Tgt->Alloc) continue;
    int32_t dX = (int32_t)Tgt->Pos.X - Look.Pos.X;
    int32_t dY = (int32_t)Tgt->Pos.Y - Look.Pos.Y;
    int32_t Dist = Acft_RelPos::FastDistance((int16_t)dX, (int16_t)dY);
    int32_t Fwd = (dX*Cos + dY*Sin + 0x800)>>12;
    int32_t Right = (dY*Cos - dX*Sin + 0x800)>>12;
    int16_t X, Y;
    if(Dist>MaxDist && Dist>0)
    { X = TrafficMapCenterX + (Right*TrafficMapRadius)/Dist;
      Y = TrafficMapCenterY - (Fwd*TrafficMapRadius)/Dist; }
    else
    { X = TrafficMapCenterX + (Right*TrafficMapRadius)/MaxDist;
      Y = TrafficMapCenterY - (Fwd*TrafficMapRadius)/MaxDist; }
    DrawTrafficTarget(X, Y, Tgt->Pos.Heading-Heading, Tgt->WarnLevel); }
  DrawTrafficAlert(Heading);
#endif
}

static bool UpdateTrafficMap(void)
{ bool GPSLock = hasStableGPSLock();
  if(GPSLock!=PrevGPSLock)
  { EPD_DrawID();
    return 1; }
  if(!GPSLock) return 0;

  uint32_t NewHash = CalcTrafficMapHash();
  if(NewHash==TrafficMapHash) return 0;
  uint32_t msTime = millis();
  uint32_t MinPeriod = 3000;
#ifdef WITH_LOOKOUT
  if(Look.WarnLevel!=TrafficMapWarn) MinPeriod=0;
  else if(Look.WarnLevel) MinPeriod=500;
#endif
  if(msTime-TrafficMapTime<MinPeriod) return 0;
  TrafficMapHash=NewHash;
  TrafficMapTime=msTime;
#ifdef WITH_LOOKOUT
  TrafficMapWarn=Look.WarnLevel;
#endif

  EPD.setPartialWindow(TrafficMapX, TrafficMapY, TrafficMapW, TrafficMapH);
  EPD.firstPage();
  EPD.fillRect(TrafficMapX, TrafficMapY, TrafficMapW, TrafficMapH, GxEPD_WHITE);
  DrawTrafficMap();
  EPD.nextPage();
  return 1; }

void EPD_TrafficRange_Next(void)
{ TrafficMapRangeIdx++;
  if(TrafficMapRangeIdx>=sizeof(TrafficMapRange)/sizeof(TrafficMapRange[0])) TrafficMapRangeIdx=0;
  TrafficMapHash=~CalcTrafficMapHash();
  TrafficMapTime=0; }

// ========================================================================================================================

static uint32_t UpdateTime = 0;
static uint32_t RedrawTime = 0;
static uint8_t PartUpd = 0;

void EPD_DrawID(void)
{ char Line[40];
  EPD.setFullWindow();                                           // this will be full page update
  EPD.firstPage();
  EPD.fillScreen(GxEPD_WHITE);                                   // all-white screen
  PrevGPSLock=hasStableGPSLock();
  if(PrevGPSLock) DrawTrafficMap();
            else { DrawLogo(); DrawLogoCopyright(); }
  EPD.setTextColor(GxEPD_BLACK);
  EPD.setFont(&FreeMonoBold9pt7b);                               // use bold font: more readable
  sprintf(Line, "%X:%d:%06X %s", Parameters.AcftType, Parameters.AddrType, Parameters.Address, Parameters.Reg);
  // sprintf(Line, "%s:%d:%06X %s", Parameters.AcftTypeName(), Parameters.AddrType, Parameters.Address, Parameters.Reg);
  EPD.setCursor(0, 195);
  EPD.print(Line);
  // drawSpeaker(110, 16, 32, GxEPD_BLACK);
  DrawAcftCount();
  DrawAlarmThresh();
  DrawBattFrame();
  EPD.nextPage();                                                // put full page onto the e-paper (takes 2 sec)
  UpdateTime = millis();
  RedrawTime=UpdateTime;
  TrafficMapTime=UpdateTime;
  TrafficMapHash=CalcTrafficMapHash();
#ifdef WITH_LOOKOUT
  TrafficMapWarn=Look.WarnLevel;
#endif
  PartUpd=0; }

void EPD_UpdateID(void)
{ uint32_t msTime=millis();
  uint32_t msAge = msTime-RedrawTime;
  if(PartUpd>25 && msAge>=300000) EPD_DrawID();                                // redraw every 10 minutes
  else
  { msAge = msTime-UpdateTime;
    if(msAge<1000) return; }                                     // do not update more frequent than once per 2 seconds
  PartUpd+=UpdateAlarmThresh();
  PartUpd+=UpdateAcftCount();
  PartUpd+=UpdateBatt();
  PartUpd+=UpdateSatMon();
  PartUpd+=UpdateTrafficMap();
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
