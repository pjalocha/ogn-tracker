#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <stdlib.h>

#include "ogn.h"
#include "ogn1.h"
#include "adsl.h"
#include "flarm.h"
#include "paw.h"

#include "rx-pkt.h"

#include "nmea.h"

// =========================================================================================================

static int ReadHex(uint8_t *Data, int MaxBytes, const char *Inp) // read from an ASCII string
{ uint8_t Len=0;
  uint8_t InpLen=0;
  for( ; Len<MaxBytes; )
  { char Char = Inp[InpLen++]; if(Char==' ') continue;
    int8_t Upp=Read_Hex1(Char); if(Upp<0) break;
    Char = Inp[InpLen++];
    int8_t Low=Read_Hex1(Char); if(Low<0) break;
    Data[Len++] = (Upp<<4) | Low; }
  // printf("ReadHex( , %d, \"%s\") => %d\n", MaxBytes, Inp, Len);
  return Len; }

static uint32_t FileTime=0;
static char     FileTimeAsc[16];
static char Line[640];
static NMEA_RxMsg NMEA;
static GPS_Position Position;

static char TmpLine[640];

static void ProcessNMEA(const char *Line)
{ int Ret=NMEA.ProcessLine(Line);
  // printf("(%3d:%c:%2d) %s\n", Ret, NMEA.isComplete()?'+':'-', NMEA.Parms, Line);
  if(!NMEA.isComplete()) return;
  Ret=Position.ReadNMEA(NMEA);
  if(Ret>0)
  { uint32_t PosTime = Position.getUnixTime();
    // printf("GPS:%d Time:%u\n", Ret, PosTime);
    if(PosTime>=1000000000)
    { FileTime = PosTime;
      int Len=Format_HHMMSS(FileTimeAsc, FileTime);
      FileTimeAsc[Len]=0; }
    return; }
  // if(NMEA.isPOGN())
  if(NMEA.isPFLA())
  { printf("%s: %s\n", FileTimeAsc, Line); }
  // if(NMEA.isP())
}

ADSL_Packet               ADSL_RxPkt;
OGN_RxPacket<OGN1_Packet> OGN_RxPkt;
Flarm_Packet              FLR_RxPkt;
PAW_Packet                PAW_RxPkt;

static void ProcessRxPkt(const char *Line)
{ if(Line[11]!=':' || Line[17]!='[' || Line[19]!=':') return;
  uint32_t RxTime=atol(Line+1); if(RxTime==0) return;
   int16_t msTime=atol(Line+12);
  uint8_t SysID = atol(Line+18);
  uint8_t PktLen = atol(Line+20);
  const char *Data=strstr(Line, "dBm "); if(Data==0) return;
  Data+=4;
  int Read=0;
  bool GoodCRC=0;
  bool isPAW=0;
  uint32_t Address=0;
  uint8_t AddrType=0;
  int32_t Altitude=0;
  if(SysID==Radio_SysID_ADSL || SysID==Radio_SysID_LDR || SysID==Radio_SysID_HDR)
  { if(PktLen<24 || PktLen>25) return;
    Read=ReadHex((uint8_t *)&(ADSL_RxPkt.Version), ADSL_RxPkt.TxBytes-2, Data);
    GoodCRC = ADSL_RxPkt.checkCRC24()==0x000000;
    if(!GoodCRC && SysID==Radio_SysID_LDR && Read==25)
    { isPAW=ADSL_RxPkt.checkCRC8()==0x00;
      PAW_RxPkt.Read(&(ADSL_RxPkt.Version));
      PAW_RxPkt.Whiten();
      GoodCRC=PAW_RxPkt.IntCRC()==0x00; }
    if(GoodCRC && !isPAW)
    { if(ADSL_RxPkt.getEncrKey()==0) ADSL_RxPkt.Descramble();
      Address=ADSL_RxPkt.getAddress();
      AddrType=ADSL_RxPkt.getAddrType();
      Altitude=ADSL_RxPkt.getAlt(); }
    ///
  }
  if(SysID==Radio_SysID_OGN)
  { if(PktLen!=26) return;
    Read=ReadHex(OGN_RxPkt.Byte(), OGN_RxPkt.Bytes, Data);
    GoodCRC = OGN_RxPkt.checkFEC()==0;
    Address=OGN_RxPkt.Packet.Header.Address;
    AddrType=OGN_RxPkt.Packet.Header.AddrType+4;
    if(GoodCRC && !OGN_RxPkt.Packet.Header.Encrypted)
    { OGN_RxPkt.Packet.Dewhiten();
      Altitude=OGN_RxPkt.Packet.DecodeAltitude();
      int Len=OGN_RxPkt.Packet.Print(TmpLine);
      printf("%s: %s\n", FileTimeAsc, TmpLine);
    }
  }
  if(SysID==Radio_SysID_FLR)
  { if(PktLen!=26) return;
    Read=ReadHex(FLR_RxPkt.Byte, FLR_RxPkt.Bytes+2, Data);
    GoodCRC = FLR_RxPkt.checkCRC()==0x0000;
    Address=FLR_RxPkt.FAMP.Address;
    AddrType=FLR_RxPkt.FAMP.AddrType+4;
    FLR_RxPkt.Time = FileTime;
    if(GoodCRC)
    { FLR_RxPkt.FAMP.Decrypt(FLR_RxPkt.Nonce, FLR_RxPkt.Time);
      if(FLR_RxPkt.FAMP.MsgType==2) Altitude=FLR_RxPkt.FAMP.getAltitude(); }
  }
  printf("%s: RxPkt: %u:%4d %d:%d [%d] CRC:%d %02X:%06X %4dm\n",
     FileTimeAsc, RxTime, msTime, SysID, PktLen, Read, GoodCRC, AddrType, Address, Altitude);
}

static void ProcessTxPkt(const char *Line)
{ if(Line[11]!=':' || Line[17]!='[' || Line[19]!=':') return;
  uint32_t TxTime=atol(Line+1); if(TxTime==0) return;
   int16_t msTime=atol(Line+12);
  uint8_t SysID = atol(Line+18);
  uint8_t PktLen = atol(Line+20);
  const char *Data=strstr(Line, "dBm "); if(Data==0) return;
  Data+=4;
  printf("%s: TxPkt: %u:%4d %d:%d\n", FileTimeAsc, TxTime, msTime, SysID, PktLen); }

// =========================================================================================================

const int MaxLineLen = 256;
static char InpLine[MaxLineLen];

static int ProcessFile(FILE *InpFile)
{ int Lines=0;
  bool TooLong=0;
  for( ; ; )
  { if(fgets(InpLine, MaxLineLen-1, InpFile)==0) break;
    char *EOL = strchr(InpLine, '\n'); if(EOL==0) { TooLong=1; continue; }
    *EOL=0; if(TooLong) { TooLong=0; continue; }
    Lines+=1;
    char Head = InpLine[0];
    if(Head=='$') { ProcessNMEA(InpLine);  continue; }
    if(Head=='>') { ProcessRxPkt(InpLine); continue; }
    if(Head=='<') { ProcessTxPkt(InpLine); continue; }
  }
  return Lines; }

// =========================================================================================================

int main(int argc, char *argv[])
{

  FILE *InpFile=stdin;
  if(argc>1)
  { InpFile=fopen(argv[1], "rt");
    if(InpFile==0) { printf("Cannot open %s for read\n", argv[1]); return -1; }
  }

  ProcessFile(InpFile);

  if(InpFile!=stdin) fclose(InpFile);
  return 0; }

// =========================================================================================================

