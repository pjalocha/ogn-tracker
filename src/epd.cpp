
#include "epd.h"

#include "proc.h"

#ifdef WITH_EPAPER

SPIClass hSPI(HSPI);                         // SPI port for the e-paper (not to conflict with Radio SPI

GxEPD2_BW<GxEPD2_154_D67, GxEPD2_154_D67::HEIGHT> EPD(GxEPD2_154_D67(EPD_PinCS, EPD_PinDC, EPD_PinRST, EPD_PinBUSY));

#include "OGN_Logo_200x200.xbm"              // OGN logo

static uint8_t ReverseBits(uint8_t X)        // reverse bytes in the OGN logo, apparently they have wrong bit order
{ X = ( X      >>4) | ( X      <<4);
  X = ((X&0xCC)>>2) | ((X&0x33)<<2);
  X = ((X&0xAA)>>1) | ((X&0x55)<<1);
  return X; }

void EPD_Init(void)                         // start the e-paper display
{ hSPI.begin(EPD_PinSCK, EPD_PinMISO, EPD_PinMOSI, EPD_PinCS);              // separate SPI not to conflict with Radio
  EPD.epd2.selectSPI(hSPI, SPISettings(4000000, MSBFIRST, SPI_MODE0));
  EPD.init(115200, true, 10, true);                                         // 10 ms reset
  EPD.setRotation(0); }

static uint8_t  PrevBattLev = 0;
static uint32_t UpdTime = 0;
static uint32_t RedrawTime = 0;

void EPD_DrawID(void)
{ char Line[32];
  EPD.setFullWindow();                                           // this will be full page update
  EPD.firstPage();
  EPD.fillScreen(GxEPD_WHITE);                                   // all-white screen
  { int Size=OGN_Logo_200x200_width*OGN_Logo_200x200_height/8;   // [bytes] size of the OGN Logo file
    uint8_t *BitMap = (uint8_t *)malloc(Size);                   // allocate bitmap with reversed bytes
    for(int Idx=0; Idx<Size; Idx++)                              // there seemed to be no other way
    { BitMap[Idx]=ReverseBits(OGN_Logo_200x200[Idx]); }          // but to manually reverse bits in every byte
    EPD.drawBitmap(0, 0, BitMap, OGN_Logo_200x200_width, OGN_Logo_200x200_height, GxEPD_BLACK);
    free(BitMap); }                                              // free the allocated bitmap
  EPD.setTextColor(GxEPD_BLACK);
  EPD.setFont(&FreeMonoBold9pt7b);                               // use bold font: more readable
  sprintf(Line, "%X:%d:%06X %s", Parameters.AcftType, Parameters.AddrType, Parameters.Address, Parameters.Reg);
  EPD.setCursor(0, 195);
  EPD.print(Line);
  EPD.drawRect(149, 0, 50, 21, GxEPD_BLACK);                     // draw empty battery symbol
  EPD.fillRect(145, 5,  5, 10, GxEPD_BLACK);
  EPD.nextPage();                                                // put full page onto the e-paper (takes 2 sec)
  UpdTime = millis(); RedrawTime=UpdTime; PrevBattLev=0; }

void EPD_UpdateID(void)
{ char Line[16];

  uint32_t msTime=millis();
  uint32_t msAge = msTime-UpdTime;
  if(msAge<2000) return;                                         // do not update more frequent than once per 2 seconds

  int16_t BattVolt=(BatteryVoltage+128)>>8;                      // [mV] measured and averaged  battery voltage
  int16_t BattLev=(BattVolt-3300)/8;
  if(BattLev<0) BattLev=0;
  else if(BattLev>100) BattLev=100;
  if(BattLev==PrevBattLev) return;

  EPD.setPartialWindow(145, 0, 55, 21);                          // partial update: the inside of the battery box
  EPD.fillRect(149, 0, 50, 21, GxEPD_WHITE);
  EPD.setTextColor(GxEPD_BLACK);
  EPD.setFont(&FreeMonoBold9pt7b);                               // use bold font: more readable
  EPD.setCursor(154, 15);
  sprintf(Line, "%3d%%", BattLev);
  EPD.drawRect(149, 0, 50, 21, GxEPD_BLACK);                     // redraw the battery frame
  EPD.fillRect(145, 5,  5, 10, GxEPD_BLACK);
  EPD.print(Line);
  EPD.nextPage();

  UpdTime=msTime; PrevBattLev=BattLev; }

#endif
