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

#include "lookout.h"

static uint32_t FileTime=0;
static char     FileTimeAsc[16];
static GPS_Position Position;

static char TmpLine[640];

// =========================================================================================================

static LookOut<32> Look;

static int ProcOwnPacket(OGN1_Packet &OwnPkt)
{ const LookOut_Target *Tgt=Look.ProcessOwn(OwnPkt, FileTime, Position.GeoidSeparation/10);
  if(Tgt && Look.WarnLevel)
  { Look.Print(); }
  return 0; }

static int ProcRxPacket(OGN1_Packet &RxPkt, uint8_t RxChan, float RxRSSI)
{ int Len=RxPkt.Print(TmpLine);
  const LookOut_Target *Tgt=Look.ProcessTarget(RxPkt, FileTime);
  Len+=sprintf(TmpLine+Len, " %+5.1fdBm/%d", RxRSSI, RxChan);
  printf("%s: %s\n", FileTimeAsc, TmpLine);
  return 0; }

static int ProcRxPacket(ADSL_Packet &RxPkt, uint8_t RxChan, float RxRSSI)
{ int Len=RxPkt.Print(TmpLine);
  const LookOut_Target *Tgt=Look.ProcessTarget(RxPkt, FileTime);
  Len+=sprintf(TmpLine+Len, " %+5.1fdBm/%d", RxRSSI, RxChan);
  printf("%s: %s\n", FileTimeAsc, TmpLine);
  return 0; }

static int ProcRxPacket(PAW_Packet &RxPkt, uint8_t RxChan, float RxRSSI)
{ if(!RxPkt.isPos()) return 0;
  OGN1_Packet Packet;
  RxPkt.Write(Packet);
  Packet.Position.Time = FileTime%60;
  ProcRxPacket(Packet, RxChan, RxRSSI);
  return 0; }

static int FLR2ADSL(ADSL_Packet &ADSL, Flarm_Packet &FLR, int32_t RefLat, int32_t RefLon)
{ if(FLR.FAMP.MsgType!=2) return 0;
  FLR.FAMP.Decrypt(FLR.Nonce, FLR.Time);     // decrypt FAMP packet based on the Time
  ADSL.Init();
  ADSL.setAddrTable(FLR.FAMP.AddrType+4);    // address-type
  ADSL.setAddress(FLR.FAMP.Address);         // address
  ADSL.setAcftTypeOGN(FLR.FAMP.AcftType);    // [aircraft-type]
  int8_t qSec=0;
  uint32_t PosTime=FLR.FAMP.getPosTime(qSec, FLR.Time);  // here we could check if PosTime==FLR.Time
  if(qSec!=0 || (PosTime!=FLR.Time && PosTime!=FLR.Time+1)) return 0;
  ADSL.TimeStamp=((PosTime%15)<<2)+qSec;        // [1/4 sec]
  ADSL.setAlt(FLR.FAMP.getAltitude());          // [m] HAE
  int32_t Lat = FLR.FAMP.getLatitude(RefLat);
  int32_t Lon = FLR.FAMP.getLongitude(RefLon, Lat);
  ADSL.setLatUBX(Lat);                            // [FNT] <= [UBX]
  ADSL.setLonUBX(Lon);                            // [FNT] <= [UBX]
  ADSL.setClimb((FLR.FAMP.getClimb()*4+2)/5);     // [0.125 m/s] <= [0.1 m/s]
  ADSL.setSpeed((FLR.FAMP.getSpeed()*2+2)/5);     // [0.250 m/s] <= [0.1 m/s]
  ADSL.setTrack((FLR.FAMP.Track*0x20+20)/45);     // [9-bit cordic] <= [0.5 deg]
  ADSL.SourceIntegrity = FLR.FAMP.SIL;
  ADSL.DesignAssurance = FLR.FAMP.SDA;
  ADSL.NavigIntegrity  = FLR.FAMP.NIC+1;
  // ADSL.HorizAccuracy FLR.FAMP.getHorPrec();  // those are coded
  // ADSL.VertAccuracy FLR.FAMP.getVerPrec();
  // ADSL.VelAccuracy FLR.FAMP.getVelPrec();
  return 1; }

static int ProcRxPacket(Flarm_Packet &RxPkt, uint8_t RxChan, float RxRSSI)
{ if(RxPkt.FAMP.MsgType!=2) return 0;
  if(!Position.isValid()) return 0;
  ADSL_Packet Packet;
  if(FLR2ADSL(Packet, RxPkt, Position.Latitude/3*50, Position.Longitude/3*50)<=0) return 0;
  ProcRxPacket(Packet, RxChan, RxRSSI);
  return 0; }

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

static NMEA_RxMsg NMEA;

static OGN1_Packet OwnPacket;

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
    if(NMEA.isGxGSA() && Position.isValid())
    { Position.Encode(OwnPacket);
      ProcOwnPacket(OwnPacket); }
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
  const char *Hash = strchr(Line+22, '#'); if(Hash==0) return;
  uint8_t RxChan = atoi(Hash+1);
  float   RxRSSI = atof(Hash+3);
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
      GoodCRC=PAW_RxPkt.IntCRC()==0x00;
      if(GoodCRC) ProcRxPacket(PAW_RxPkt, RxChan, RxRSSI);
    }
    if(GoodCRC && !isPAW)
    { if(ADSL_RxPkt.getEncrKey()==0) ADSL_RxPkt.Descramble();
      Address=ADSL_RxPkt.getAddress();
      AddrType=ADSL_RxPkt.getAddrType();
      Altitude=ADSL_RxPkt.getAlt();
      ProcRxPacket(ADSL_RxPkt, RxChan, RxRSSI); }
  }
  else if(SysID==Radio_SysID_OGN)
  { if(PktLen!=26) return;
    Read=ReadHex(OGN_RxPkt.Byte(), OGN_RxPkt.Bytes, Data);
    GoodCRC = OGN_RxPkt.checkFEC()==0;
    Address=OGN_RxPkt.Packet.Header.Address;
    AddrType=OGN_RxPkt.Packet.Header.AddrType+4;
    if(GoodCRC && !OGN_RxPkt.Packet.Header.Encrypted)
    { OGN_RxPkt.Packet.Dewhiten();
      Altitude=OGN_RxPkt.Packet.DecodeAltitude();
      ProcRxPacket(OGN_RxPkt.Packet, RxChan, RxRSSI); }
  }
  else if(SysID==Radio_SysID_FLR)
  { if(PktLen!=26) return;
    Read=ReadHex(FLR_RxPkt.Byte, FLR_RxPkt.Bytes+2, Data);
    GoodCRC = FLR_RxPkt.checkCRC()==0x0000;
    FLR_RxPkt.Time = FileTime;
    if(GoodCRC)
    { ProcRxPacket(FLR_RxPkt, RxChan, RxRSSI); }
  }
  // printf("%s: RxPkt: %u:%4d %d:%d #%d %+6.1fdBm [%d] CRC:%d %02X:%06X %4dm\n",
  //    FileTimeAsc, RxTime, msTime, SysID, PktLen, RxChan, RxRSSI, Read, GoodCRC, AddrType, Address, Altitude);
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

const int MaxLineLen = 1024;
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
