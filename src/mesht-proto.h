#pragma once

#include <stdio.h>
#include <stdint.h>
#include <inttypes.h>

#include "format.h"
// #include "aircraft.h"

class MeshtProto_NodeInfo
{ public:
   char Name [64];
   char Short[8];
   uint8_t Hardware; // src/mesh/generated/meshtastic/mesh.pb.h
   uint8_t Role;     // 5:tracker

  public:
   MeshtProto_NodeInfo() { Clear(); }
   void Clear(void) { Name[0]=0; Short[0]=0; Role=0; Hardware=0; }

   const char *RoleNameShort(void) const
   { static const char *Table[13] = { "Cnt", "C/m", "Rtr", "R+C", "Rpt", "Tkr", "Ssr", "TAK",
                                      "C/h", "L&F", "TTr", "R/l", "C+B" } ;
     if(Role<13) return Table[Role];
     return "-?-"; }

   const char *RoleName(void) const
   { static const char *Table[13] = { "Client", "Client/mute", "Router", "Router+Client", "Repeater", "Tracker", "Sensor", "TAK",
                                      "Client/hidden", "Lost&Found", "TAK+Tracker", "Router/late", "Client+Base" } ;
     if(Role<13) return Table[Role];
     return "-?-"; }

} ;

class MeshtProto_TelemDev
{ public:
   uint16_t BattVolt;   // [mV]
   uint8_t  BattFull;   // [%]
   uint8_t  UpTime;     // [hours]

  public:

} ;

class MeshtProto_GPS
{ public:
  uint32_t  SensorID;
  uint32_t  Time;
   int32_t  Lat;
   int32_t  Lon;
   int32_t  AltHAE;
   int32_t  AltMSL;
   uint16_t Speed;
   uint16_t Track;
    int8_t  GeoidSepar;
   uint8_t  Prec_bits;
   uint8_t  Prec_meters;
   uint8_t  PDOP;
   uint8_t  HDOP;
   uint8_t  VDOP;
   union
   { uint16_t  Flags;
     struct
     { uint8_t PosSrc    : 2;
       uint8_t AltSrc    : 3;
       bool hasAltHAE    : 1;
       bool hasAltMSL    : 1;
       bool hasGeoidSepar: 1;
       bool hasSpeed     : 1;
       bool hasTrack     : 1;
       bool hasSensorID  : 1;
       uint8_t Sats      : 5;
     } ;
   } ;

  public:
   MeshtProto_GPS() { Clear(); }
   void Clear(void)
   { SensorID=0;
     Flags=0; Time=0; Lat=Lon=0x7FFFFFFF;
     Prec_bits=0; Prec_meters=255;
     PDOP=255; HDOP=255; VDOP=255; }

   bool hasTime(void) const { return Time>0; }
   bool hasLat(void) const { return Lat>=( -900000000) && Lat<= 900000000; }
   bool hasLon(void) const { return Lon>=(-1800000000) && Lon<=1800000000; }
   bool hasCoord(void) const { return hasLat() && hasLon(); }

   int32_t getAltMSL(void) const
   { if(hasAltMSL) return AltMSL;
     if(hasAltHAE && hasGeoidSepar) return AltHAE-GeoidSepar;
     return 0; }

   unsigned PrecBitsPosError(void) const      // est. position error due to (intentional) accuracy reduction
   { if(Prec_bits>=28) return 0;
     const float DistPerBit = 40e6/360/1e7;          // [m]
     uint64_t MultFactor=1; MultFactor<<=(32-Prec_bits);
     return roundf(DistPerBit*MultFactor); }

#ifdef OBSOLETE
   int WriteAPRS(char *Line, const char *Icon, bool HighRes=1) const
   { int Len=0;

     Line[Len++]='/';
     if(hasTime()) Len+=Format_HHMMSS(Line+Len, Time);             // time in HHMMSS UTC
             else  Len+=Format_String(Line+Len, "______");
     Line[Len++] ='h';

     char LatHighRes, LonHighRes;
     Len+=Acft_TimePosSpeed::Format_Lat(Line+Len, Lat, LatHighRes); // Latitude
     Line[Len++] = Icon[0];                                         // first icon character
     Len+=Acft_TimePosSpeed::Format_Lon(Line+Len, Lon, LonHighRes); // Longitude
     Line[Len++] = Icon[1];                                         // second icon character

     if(hasSpeed && hasTrack && Speed>0)
     { uint32_t Degrees = (Track+50)/100;
       Len+=Format_UnsDec(Line+Len, Degrees, 3);
       Line[Len++] = '/';
       uint32_t Knots = (1944*(uint32_t)Speed+500)/1000;
       Len+=Format_UnsDec(Line+Len, Knots, 3); }

     int Alt=-1;
     if(hasAltMSL) Alt=AltMSL;
     else if(hasAltHAE && hasGeoidSepar) Alt=AltHAE-GeoidSepar;

     if(Alt>=0)
     { uint32_t Feet = (3360*Alt+512)>>10;
       Line[Len++]='/'; Line[Len++]='A'; Line[Len++]='=';
       Len+=Format_UnsDec(Line+Len, Feet, 6); }

     if(HighRes)
     { Line[Len++]=' '; Line[Len++]='!'; Line[Len++]= 'W'; Line[Len++]=LatHighRes; Line[Len++]=LonHighRes; Line[Len++]='!'; }

     uint32_t PosErr=PrecBitsPosError();
     if(PosErr>10)
     { Line[Len++]=' '; Line[Len++]='<';
       if(PosErr<100) Len+=Format_UnsDec(Line+Len, PosErr);
       else { Len+=Format_UnsDec(Line+Len, (PosErr+50)/100, 2, 1); Line[Len++]='k'; }
       Line[Len++]='m'; Line[Len++]='>'; }

     if(Prec_meters<255)
     { // Len+=Format_String(Line+Len, " gps");
       // Len+=Format_;
     }

     Line[Len]=0; return Len; }

   uint32_t getDist(int32_t RefLat, int32_t RefLon)
   { int32_t LatDist=Acft_TimePos::LatDist(Lat, RefLat);
     int32_t LonDist=Acft_TimePos::LonDist(Lat, RefLat, Acft_TimePos::getLatCos(Lat));
     return IntDistance(LatDist, LonDist); }

#endif // of OBSOLETE
} ;


// Protobuf decoding (meant for Meshtantic)
class MeshtProto
{ public:

  // Read the Key, return ID and Wire
  static int ReadKey(uint64_t &ID, uint8_t &Wire, const uint8_t *Inp, int InpLen)
  { Wire=8;
    int Len=ReadVarInt(ID, Inp, InpLen); if(Len<=0) return Len;
    Wire = ID&7;
    ID>>=3;
    // printf("ReadKey() ID:0x%lX Wire:%d Len:%d\n", ID, Wire, Len);
    return Len; }  // return number of bytes read from the Inp

  // Read the key, assuming the ID is a single byte (mot are, but not all)
  static int ReadKey(uint8_t &ID, uint8_t &Wire, const uint8_t *Inp, int InpLen)
  { uint64_t LongID=0;
    int Len=ReadKey(LongID, Wire, Inp, InpLen); if(Len<=0) return Len;
    if(LongID>0xFF) return -1;
    ID=LongID;
    return Len; }

  template <class Type>
   static int WriteKey(uint8_t *Out, Type ID, uint8_t Wire)
  { ID<<=3; ID|=Wire; return WriteVarInt(Out, ID); }

  template <class Type>
   static int WriteVarInt(uint8_t *Out, Type Value)
  { int Len=0;
    for( ; ; )
    { uint8_t Byte=Value&0x7F;
      Value>>=7;
      if(Value>0) Byte|=0x80;
      Out[Len++]=Byte;
      if(Value==0) break; }
    return Len; }

  // Wire-types
  static const uint8_t Wire_VarInt = 0;   // variable-size integer
  static const uint8_t Wire_Int64  = 1;   // 64-bit = 8-byte integer (or double)
  static const uint8_t Wire_Bytes  = 2;   // given number of bytes
  static const uint8_t Wire_Int32  = 5;   // 32-bit = 4-byte integer (or float)

  static uint8_t getWire(const uint8_t *Inp) { return Inp[0]&7; }         // extract Wire from the first byte of Inp
  static bool isWireVarInt(uint8_t Wire) { return Wire==Wire_VarInt; }
  static bool isWireInt64 (uint8_t Wire) { return Wire==Wire_Int64;  }
  static bool isWireInt32 (uint8_t Wire) { return Wire==Wire_Int32;  }
  static bool isWireBytes (uint8_t Wire) { return Wire==Wire_Bytes;  }

  // read variable-size integer (up to 4 bytes)
  static int ReadVarInt(uint32_t &Value, const uint8_t *Inp, int InpLen)
  { uint64_t LongValue=0;
    int Len=ReadVarInt(LongValue, Inp, InpLen); if(Len<=0) return Len;
    if(LongValue>0xFFFFFFFF) return 0;
    // printf("ReadVarInt(uint32_t, ) => %08lX\n", LongValue);
    Value=LongValue; return Len; }

  // read variable-size integer (up to 8 bytes)
  static int ReadVarInt(uint64_t &Value, const uint8_t *Inp, int InpLen)
  { int Len=0; int Shift=0; Value=0;
    for( ; ; )
    { if(Len>=InpLen || Shift>=63) return -1;
      uint64_t Byte=Inp[Len++];
      Value |= (Byte&0x7F)<<Shift; Shift+=7;
      if((Byte&0x80)==0) break; }
    // printf("ReadVarInt() Value:0x%lX Shift:%d Len:%d\n", Value, Shift, Len);
    return Len; }

  static int WriteInt64(uint8_t *Out, uint64_t &Value)
  { int Len=0;
    for( ; Len<8; Len++)
    { Out[Len]=Value; Value>>=8; }
    return Len; }

  // read 8-byte integer, wire #1
  static int ReadInt64(uint64_t &Value, const uint8_t *Inp, int InpLen)
  { int Len=0; int Shift=0; Value=0;
    for( ; Len<8; )
    { if(Len>=InpLen) return -1;
      uint64_t Byte=Inp[Len++];
      Value |= Byte<<Shift; Shift+=8; }
    // printf("ReadInt64() Value:0x%lX Shift:%d Len:%d\n", Value, Shift, Len);
    return Len; }

  static int WriteInt32(uint8_t *Out, uint32_t Value)
  { int Len=0;
    for( ; Len<4; Len++)
    { Out[Len]=Value; Value>>=8; }
    return Len; }

  // read 4-byte integer as signed, wire #5
  static int ReadInt32(int32_t &Value, const uint8_t *Inp, int InpLen)
  { return ReadInt32((uint32_t &)Value, Inp, InpLen); }

  // read 4-byte integer, wire #5
  static int ReadInt32(uint32_t &Value, const uint8_t *Inp, int InpLen)
  { // printf(" -> ReadInt32( , , %d)\n", InpLen);
    int Len=0; int Shift=0; Value=0;
    for( ; Len<4; )
    { if(Len>=InpLen) return -1;
      uint32_t Byte=Inp[Len++];
      Value |= Byte<<Shift; Shift+=8; }
    // printf("ReadInt32() Value:0x%X Shift:%d Len:%d\n", Value, Shift, Len);
    return Len; }

  static int WriteBytes(uint8_t *Out, const uint8_t *Byte, int Bytes)
  { int Len=0;
    for( ; Len<Bytes; Len++)
    { Out[Len]=Byte[Len]; }
    return Len; }

  // dump elements of a packet
  static int Dump(const uint8_t *Inp, int InpLen)
  { uint8_t ID, Wire;
    for( ; InpLen>0; )
    { int Len=ReadKey(ID, Wire, Inp, InpLen); if(Len<=0) break;
      Inp+=Len; InpLen-=Len;
      if(Wire==0)
      { uint64_t Int;
        int Len=ReadVarInt(Int, Inp, InpLen); if(Len<=0) break;
        Inp+=Len; InpLen-=Len;
        printf("%2d:VarInt: 0x%" PRIX64 " = %" PRId64 "\n", ID, Int, Int); continue; }
      if(Wire==1)
      { uint64_t Int;
        int Len=ReadInt64(Int, Inp, InpLen); if(Len<=0) break;
        Inp+=Len; InpLen-=Len;
        printf("%2d:Int64 : 0x%016" PRIX64 " = %20" PRId64 "\n", ID, Int, Int); continue; }
      if(Wire==5)
      { uint32_t Int;
        int Len=ReadInt32(Int, Inp, InpLen); if(Len<=0) break;
        Inp+=Len; InpLen-=Len;
        printf("%2d:Int32 : 0x%08X = %10d\n", ID, Int, Int); continue; }
      if(Wire==2)
      { uint64_t Int;
        int Len=ReadVarInt(Int, Inp, InpLen); if(Len<=0) break;
        Inp+=Len; InpLen-=Len;
        printf("%2d:[%4" PRId64 "]: ", ID, Int);
        for(uint64_t Idx=0; Idx<Int && InpLen>0; Idx++)
        { printf("%02X", *Inp++); InpLen--; }
        printf("\n");
        continue; }
      printf(" Wire #%d ?\n", Wire);
      break;
    }
    return InpLen; }

  // in src/mesh/generated/meshtastic/portnums.pb.h
  // port numbers: the first VarInt of the packet on the air
  static const uint8_t Port_TextMsg   =  1;
  static const uint8_t Port_GPSpos    =  3;
  static const uint8_t Port_NodeInfo  =  4;
  static const uint8_t Port_Telemetry = 67;

  static int ReadMsgPort(uint64_t &Port, const uint8_t *Inp, int InpLen)
  { Port=0;
    uint8_t ID, Wire;
    int Len=ReadKey(ID, Wire, Inp, InpLen); if(Len<=0) return Len;      //
    if(!isWireVarInt(Wire) || ID!=1) return 0;                          // the first element should be VarInt and ID=1
    int Len2=ReadVarInt(Port, Inp+Len, InpLen-Len); if(Len2<=0) return Len2;  // read VarInt: it is the Port number
    return Len+Len2; }

  static int DecodeMsg(char *Out, const uint8_t *Inp, int InpLen)
  { int OutLen=0;
    uint64_t Port=0;
    int Len=ReadMsgPort(Port, Inp, InpLen); if(Len<=0) return Len;
    Inp+=Len; InpLen-=Len;
    if(Port==Port_TextMsg)   return DecodeTextMsg  (Out, Inp, InpLen);  // decode depending on the Port
    if(Port==Port_GPSpos)    return DecodeGPSpos   (Out, Inp, InpLen);
    if(Port==Port_Telemetry) return DecodeTelemetry(Out, Inp, InpLen);
    if(Port==Port_NodeInfo)  return DecodeNodeInfo (Out, Inp, InpLen);
    // other ports ?
    return OutLen; }

  static int DecodeTextMsg(char *Out, const uint8_t *Inp, int InpLen)
  { int OutLen=0;
    uint8_t ID, Wire;
    int Len=ReadKey(ID, Wire, Inp, InpLen); if(Len<=0) return Len;
    if(!isWireBytes(Wire) || ID!=2) return 0;
    Inp+=Len; InpLen-=Len;
    uint64_t MsgLen;
    Len=ReadVarInt(MsgLen, Inp, InpLen); if(Len<=0) return Len;
    Inp+=Len; InpLen-=Len;
    OutLen=sprintf(Out, "Msg: ");
    for(uint64_t Idx=0; Idx<MsgLen; Idx++)
    { if(InpLen<=0) break;
      uint8_t Char = *Inp++; InpLen--;
      if(Char<' ') OutLen=sprintf(Out+OutLen, "\\x%02x", Char);
              else Out[OutLen++]=Char;
    }
    Out[OutLen]=0; return OutLen; }

  // ZigZag encode and decode - to transmit negative values as VarInt
  static uint32_t ZZencode32( int32_t N) { return (uint32_t)((N<<1) ^  (N >> 31)); }
  static  int32_t ZZdecode32(uint32_t N) { return ( int32_t)((N>>1) ^ -(int32_t)(N&1)); }

  // in protobufs/meshtastic/mesh.proto
  static const uint8_t GPS_Latitude   = 1; // [UBX] uint32
  static const uint8_t GPS_Longitude  = 2; // [UBX] uint32
  static const uint8_t GPS_AltMSL     = 3; // [m] var-int
  static const uint8_t GPS_Time       = 4; // [s] uint32
  static const uint8_t GPS_PosSrc     = 5; // [] 0=unset, 1=manual, 2=internal, 3=external
  static const uint8_t GPS_AltSrc     = 6; // [] 0=unset, 1=manual, 2=internal, 3=external, 4=barometer
  static const uint8_t GPS_Timestamp  = 7; // [s] uint32
  static const uint8_t GPS_AltHAE     = 9; // [m] var-int
  static const uint8_t GPS_GeoidSepar =10; // [m] int32
  static const uint8_t GPS_PDOP       =11; // [0.01]
  static const uint8_t GPS_HDOP       =12; // [0.01]
  static const uint8_t GPS_VDOP       =13; // [0.01]
  static const uint8_t GPS_Accuracy   =14; // [m]
  static const uint8_t GPS_Speed      =15; // [m/s]
  static const uint8_t GPS_Track      =16; // [0.01deg]
  static const uint8_t GPS_FixQuality =17; // []
  static const uint8_t GPS_FixType    =18; // []
  static const uint8_t GPS_ViewSats   =19; // []
  static const uint8_t GPS_SensorID   =20; // []
  static const uint8_t GPS_SeqNum     =22; // []
  static const uint8_t GPS_PrecBits   =23; // []

  static int EncodeGPS(uint8_t *Packet, const MeshtProto_GPS &GPS)
  { int Len=0;
    Len+=WriteKey(Packet+Len, (uint8_t)1, Wire_VarInt);
    Len+=WriteVarInt(Packet+Len, Port_GPSpos);
    Len+=WriteKey(Packet+Len, (uint8_t)2, Wire_Bytes);
    Len+=WriteVarInt(Packet+Len, (uint8_t)0);
    int Start=Len;
    if(GPS.hasSensorID)
    { Len+=WriteKey(Packet+Len, GPS_SensorID, Wire_Int32);
      Len+=WriteInt32(Packet+Len, GPS.SensorID); }
    if(GPS.Time>0)
    { Len+=WriteKey(Packet+Len, GPS_Time, Wire_Int32);
      Len+=WriteInt32(Packet+Len, GPS.Time); }
    if(GPS.hasCoord())
    { Len+=WriteKey(Packet+Len, GPS_Latitude, Wire_Int32);
      Len+=WriteInt32(Packet+Len, GPS.Lat);
      Len+=WriteKey(Packet+Len, GPS_Longitude, Wire_Int32);
      Len+=WriteInt32(Packet+Len, GPS.Lon); }
    if(GPS.hasAltMSL && GPS.AltMSL>0)
    { Len+=WriteKey(Packet+Len, GPS_AltMSL, Wire_VarInt);
      Len+=WriteVarInt(Packet+Len, GPS.AltMSL); }
    if(GPS.hasAltHAE && GPS.AltHAE>0)
    { Len+=WriteKey(Packet+Len, GPS_AltHAE, Wire_VarInt);
      Len+=WriteVarInt(Packet+Len, GPS.AltHAE); }
    if(GPS.hasSpeed && GPS.Speed>0)
    { Len+=WriteKey(Packet+Len, GPS_Speed, Wire_VarInt);
      Len+=WriteVarInt(Packet+Len, GPS.Speed); }
    if(GPS.hasTrack && GPS.Speed>0)
    { Len+=WriteKey(Packet+Len, GPS_Track, Wire_VarInt);
      Len+=WriteVarInt(Packet+Len, GPS.Track); }
    if(GPS.Prec_bits>0)
    { Len+=WriteKey(Packet+Len, GPS_PrecBits, Wire_VarInt);
      Len+=WriteVarInt(Packet+Len, GPS.Prec_bits); }
    Packet[Start-1]=Len-Start;
    return Len; }

  static int DecodeGPS(MeshtProto_GPS &GPS, const uint8_t *Inp, int InpLen)
  { GPS.Clear();
    uint8_t ID, Wire;
    int Len=ReadKey(ID, Wire, Inp, InpLen); if(Len<=0) return Len;
    if(!isWireBytes(Wire) || ID!=2) return 0;                              // must be series-of-bytes type and ID=2
    Inp+=Len; InpLen-=Len;
    uint64_t MsgLen;                                                       // [bytes] GPS message size
    Len=ReadVarInt(MsgLen, Inp, InpLen); if(Len<=0) return Len;            // if not enough bytes then give up
    Inp+=Len; InpLen-=Len;
    for( ; InpLen>0; )
    { int Len=ReadKey(ID, Wire, Inp, InpLen); if(Len<=0) return Len;
      // printf("GPS: InpLen:%d ID:%d Wire:%d\n", InpLen, ID, Wire);
      Inp+=Len; InpLen-=Len;
      uint32_t Value;
            if(isWireInt32 (Wire)) Len=ReadInt32 (Value, Inp, InpLen);
      else  if(isWireVarInt(Wire)) Len=ReadVarInt(Value, Inp, InpLen);
      else break;
      if(Len<=0) return Len;
      Inp+=Len; InpLen-=Len;
      if(ID==GPS_Latitude)  { GPS.Lat         =Value; continue; }
      if(ID==GPS_Longitude) { GPS.Lon         =Value; continue; }
      if(ID==GPS_AltMSL)    { GPS.AltMSL      =Value; GPS.hasAltMSL=1; continue; }
      if(ID==GPS_AltHAE)    { GPS.AltHAE      =Value; GPS.hasAltHAE=1; continue; }
      if(ID==GPS_GeoidSepar) { GPS.GeoidSepar =Value; GPS.hasGeoidSepar=1; continue; }
      if(ID==GPS_Time || ID==GPS_Timestamp) { GPS.Time=Value; continue; }
      if(ID==GPS_SensorID)  { GPS.SensorID=Value; GPS.hasSensorID=1; continue; }
      if(ID==GPS_Speed)     { GPS.Speed =Value; GPS.hasSpeed=1; continue; }
      if(ID==GPS_Track)     { GPS.Track =Value; GPS.hasTrack=1; continue; }
      if(ID==GPS_PosSrc)    { GPS.PosSrc=Value; continue; }
      if(ID==GPS_AltSrc)    { GPS.AltSrc=Value; continue; }
      if(ID==GPS_ViewSats)  { GPS.Sats=Value; continue; }
      if(ID==GPS_PrecBits)  { GPS.Prec_bits=Value; continue; }
      if(ID==GPS_Accuracy)  { GPS.Prec_meters=Value; continue; }
    }
    return GPS.hasTime() || GPS.hasCoord(); }

  static int DecodeGPSpos(char *Out, const uint8_t *Inp, int InpLen)
  { int OutLen=0;
    uint8_t ID, Wire;
    int Len=ReadKey(ID, Wire, Inp, InpLen); if(Len<=0) return Len;
    if(!isWireBytes(Wire) || ID!=2) return 0;                              // must be series-of-bytes type and ID=2
    Inp+=Len; InpLen-=Len;
    uint64_t MsgLen;                                                       // [bytes] GPS message size
    Len=ReadVarInt(MsgLen, Inp, InpLen); if(Len<=0) return Len;            // if not enough bytes then give up
    Inp+=Len; InpLen-=Len;
    // OutLen=sprintf(Out, "GPS: [%ld]", MsgLen);
    uint32_t Time=0;
    int32_t Lat=0, Lon=0, Alt=-1;
     int8_t PrecBits=-1;
     int8_t PosSrc=-1, AltSrc=-1;
    for( ; InpLen>0; )
    { int Len=ReadKey(ID, Wire, Inp, InpLen); if(Len<=0) return Len;
      // printf("GPS: InpLen:%d ID:%d Wire:%d\n", InpLen, ID, Wire);
      Inp+=Len; InpLen-=Len;
      uint32_t Value;
            if(isWireInt32 (Wire)) Len=ReadInt32 (Value, Inp, InpLen);
      else  if(isWireVarInt(Wire)) Len=ReadVarInt(Value, Inp, InpLen);
      else break;
      if(Len<=0) return Len;
      Inp+=Len; InpLen-=Len;
      if(ID==GPS_Latitude)  { Lat =Value; continue; }
      if(ID==GPS_Longitude) { Lon =Value; continue; }
      if(ID==GPS_AltMSL)    { Alt =Value; continue; }
      if(ID==GPS_Time || ID==GPS_Timestamp) { Time=Value; continue; }
      if(ID==GPS_PosSrc)    { PosSrc=Value; continue; }
      if(ID==GPS_AltSrc)    { AltSrc=Value; continue; }
      if(ID==GPS_PrecBits)  { PrecBits=Value; continue; }
    }
    OutLen=sprintf(Out, "GPS: [%+09.5f,%+010.5f]deg", 1e-7*Lat, 1e-7*Lon);
    if(PosSrc>=0) OutLen+=sprintf(Out+OutLen, "/%d", PosSrc);
    if(Alt>=0) OutLen+=sprintf(Out+OutLen, " %dm", Alt);
    if(AltSrc>=0) OutLen+=sprintf(Out+OutLen, "/%d", AltSrc);
    if(Time>0) OutLen+=sprintf(Out+OutLen, " %10ds", Time);
    if(PrecBits>=0) OutLen+=sprintf(Out+OutLen, " %dbits", PrecBits);
    return OutLen; }

  static const uint8_t Node_ID        = 1;
  static const uint8_t Node_LongName  = 2;
  static const uint8_t Node_ShortName = 3;
  static const uint8_t Node_MAC       = 4; //
  static const uint8_t Node_Hardware  = 5;
  static const uint8_t Node_Licenced  = 6; // has license for higher limits on the band
  static const uint8_t Node_Role      = 7; // 0:client, 1:client-mute, 2:router, 3:router+client, 4:repeater, 5:tracker, 7:sensor
  static const uint8_t Node_PubKey    = 8;

  static int DecodeNodeInfo(MeshtProto_NodeInfo &Node, const uint8_t *Inp, int InpLen)
  { Node.Clear();
    uint8_t ID, Wire;
    int Len=ReadKey(ID, Wire, Inp, InpLen); if(Len<=0) return Len;
    if(!isWireBytes(Wire) || ID!=2) return 0;
    Inp+=Len; InpLen-=Len;
    uint64_t MsgLen;
    Len=ReadVarInt(MsgLen, Inp, InpLen); if(Len<=0) return Len;
    Inp+=Len; InpLen-=Len;
    for( ; InpLen>0; )
    { int Len=ReadKey(ID, Wire, Inp, InpLen); if(Len<=0) return Len;
      Inp+=Len; InpLen-=Len;
      uint32_t Value=0; const uint8_t *chVal=0; uint32_t chLen=0;
            if(isWireInt32 (Wire)) Len=ReadInt32 (Value, Inp, InpLen);
      else  if(isWireVarInt(Wire)) Len=ReadVarInt(Value, Inp, InpLen);
      else  if(isWireBytes (Wire)) { chVal=Inp; Len=ReadVarInt(chLen, Inp, InpLen); }
      else break;
      if(Len<=0) return Len;
      Inp+=Len; InpLen-=Len; if(chVal) chVal+=Len;
      if(chVal)
      { Inp+=chLen; InpLen-=chLen;
        if(ID==Node_LongName)  { if(chLen>63) chLen=63; memcpy(Node.Name , chVal, chLen); Node.Name [chLen]=0; }
        if(ID==Node_ShortName) { if(chLen> 7) chLen= 7; memcpy(Node.Short, chVal, chLen); Node.Short[chLen]=0; }
      }
      else
      { if(ID==Node_Role) Node.Role=Value;
        if(ID==Node_Hardware) Node.Hardware=Value;
      }
    }
    return 0; }

  static int DecodeNodeInfo(char *Out, const uint8_t *Inp, int InpLen)
  { int OutLen=0;
    uint8_t ID, Wire;
    int Len=ReadKey(ID, Wire, Inp, InpLen); if(Len<=0) return Len;
    if(!isWireBytes(Wire) || ID!=2) return 0;
    Inp+=Len; InpLen-=Len;
    uint64_t MsgLen;
    Len=ReadVarInt(MsgLen, Inp, InpLen); if(Len<=0) return Len;
    Inp+=Len; InpLen-=Len;
    // OutLen=sprintf(Out, "Node: [%ld]", MsgLen);
    OutLen=sprintf(Out, "Node:");
    for( ; InpLen>0; )
    { int Len=ReadKey(ID, Wire, Inp, InpLen); if(Len<=0) return Len;
      // printf("Node: InpLen:%d ID:%d Wire:%d\n", InpLen, ID, Wire);
      Inp+=Len; InpLen-=Len;
      uint32_t Value=0; const uint8_t *chVal=0; uint32_t chLen=0;
            if(isWireInt32 (Wire)) Len=ReadInt32 (Value, Inp, InpLen);
      else  if(isWireVarInt(Wire)) Len=ReadVarInt(Value, Inp, InpLen);
      else  if(isWireBytes (Wire)) { chVal=Inp; Len=ReadVarInt(chLen, Inp, InpLen); }
      else break;
      if(Len<=0) return Len;
      Inp+=Len; InpLen-=Len; if(chVal) chVal+=Len;
      if(chVal)
      { Inp+=chLen; InpLen-=chLen;
        if(ID==Node_ID)        { OutLen+=sprintf(Out+OutLen, " ID=");    OutLen+=PrintTxt(Out+OutLen, chVal, chLen); }
        if(ID==Node_LongName)  { OutLen+=sprintf(Out+OutLen, " Name=");  OutLen+=PrintTxt(Out+OutLen, chVal, chLen); }
        if(ID==Node_ShortName) { OutLen+=sprintf(Out+OutLen, " Short="); OutLen+=PrintTxt(Out+OutLen, chVal, chLen); }
        if(ID==Node_MAC)       { OutLen+=sprintf(Out+OutLen, " MAC=");   OutLen+=PrintHex(Out+OutLen, chVal, chLen); }
        if(ID==Node_PubKey)    { OutLen+=sprintf(Out+OutLen, " Key=");   OutLen+=PrintHex(Out+OutLen, chVal, chLen); }
      }
      else
      { if(ID==Node_Hardware) OutLen+=sprintf(Out+OutLen, " HW=%u", Value);
        if(ID==Node_Licenced) OutLen+=sprintf(Out+OutLen, " Licence=%u", Value);
        if(ID==Node_Role)     OutLen+=sprintf(Out+OutLen, " Role=%u", Value);
      }
    }
    return OutLen; }

  static int PrintTxt(char *Out, const uint8_t *Data, int Len)
  { for(int Idx=0; Idx<Len; Idx++)
      Out[Idx]=Data[Idx];
    return Len; }

  static int PrintHex(char *Out, const uint8_t *Data, int Len)
  { int OutLen=0;
    for(int Idx=0; Idx<Len; Idx++)
    { OutLen+=sprintf(Out+OutLen, "%02X", Data[Idx]); }
    return OutLen; }

  // in protobufs/meshtastic/telemetry.proto
  static const uint8_t Telem_Time   = 1; // [s] uint32
  static const uint8_t Telem_Dev    = 2; // [] Device metrics
  static const uint8_t Telem_Env    = 3; // [] Environment metrics
  static const uint8_t Telem_Air    = 4; // [] Air quallity metrics
  static const uint8_t Telem_Power  = 5; // [] Power metrics
  static const uint8_t Telem_Stats  = 6; // [] Local statistics
  static const uint8_t Telem_Health = 7; // [] Health metrics

  static int DecodeTelemetry(char *Out, const uint8_t *Inp, int InpLen)
  { int OutLen=0;
    uint8_t ID, Wire;
    int Len=ReadKey(ID, Wire, Inp, InpLen); if(Len<=0) return Len;
    if(!isWireBytes(Wire) || ID!=2) return 0;
    Inp+=Len; InpLen-=Len;
    uint64_t MsgLen;
    Len=ReadVarInt(MsgLen, Inp, InpLen); if(Len<=0) return Len;
    Inp+=Len; InpLen-=Len;
    // OutLen=sprintf(Out, "Tel: [%ld]", MsgLen);
    OutLen=sprintf(Out, "Telem:");
    for( ; InpLen>0; )
    { int Len=ReadKey(ID, Wire, Inp, InpLen); if(Len<=0) return Len;
      // printf("Node: InpLen:%d ID:%d Wire:%d\n", InpLen, ID, Wire);
      Inp+=Len; InpLen-=Len;
      uint32_t Value=0; const uint8_t *chVal=0; uint32_t chLen=0;
            if(isWireInt32 (Wire)) Len=ReadInt32 (Value, Inp, InpLen);
      else  if(isWireVarInt(Wire)) Len=ReadVarInt(Value, Inp, InpLen);
      else  if(isWireBytes (Wire)) { chVal=Inp; Len=ReadVarInt(chLen, Inp, InpLen); }
      else break;
      if(Len<=0) return Len;
      Inp+=Len; InpLen-=Len; if(chVal) chVal+=Len;
      if(chVal)
      { Inp+=chLen; InpLen-=chLen;
        if(ID==Telem_Dev)      { OutLen+=PrintTelem_Dev(Out+OutLen, chVal, chLen); }
        if(ID==Telem_Env)      { OutLen+=PrintTelem_Env(Out+OutLen, chVal, chLen); }
        if(ID==Telem_Air)      { OutLen+=PrintTelem_Air(Out+OutLen, chVal, chLen); }
        if(ID==Telem_Power)    { OutLen+=PrintTelem_Pwr(Out+OutLen, chVal, chLen); }
        if(ID==Telem_Stats)    { OutLen+=PrintTelem_Stat(Out+OutLen, chVal, chLen); }
        if(ID==Telem_Health)   { OutLen+=PrintTelem_Health(Out+OutLen, chVal, chLen); }
      }
      else
      { if(ID==Telem_Time) OutLen+=sprintf(Out+OutLen, " Time=%us", Value);
      }
    }
    return OutLen; }

  // device metrics
  static const uint8_t Telem_Dev_BattLevel = 1; // [%] uint 0..100
  static const uint8_t Telem_Dev_BattVolt  = 2; // [V] float
  static const uint8_t Telem_Dev_AirUtil  =  3; // [%] float
  static const uint8_t Telem_Dev_ChanUtil =  4; // [%] float
  static const uint8_t Telem_Dev_UpTime   =  5; // [s] uint

  static int PrintTelem_Dev(char *Out, const uint8_t *Inp, int InpLen)
  { int OutLen=sprintf(Out, " Dev:");;
    uint8_t ID, Wire;
    for( ; InpLen>0; )
    { int Len=ReadKey(ID, Wire, Inp, InpLen); if(Len<=0) return Len;
      Inp+=Len; InpLen-=Len;
      uint32_t Value=0; const uint8_t *chVal=0; uint32_t chLen=0;
            if(isWireInt32 (Wire)) Len=ReadInt32 (Value, Inp, InpLen);
      else  if(isWireVarInt(Wire)) Len=ReadVarInt(Value, Inp, InpLen);
      else  if(isWireBytes (Wire)) { chVal=Inp; Len=ReadVarInt(chLen, Inp, InpLen); }
      else break;
      if(Len<=0) return Len;
      Inp+=Len; InpLen-=Len; if(chVal) chVal+=Len;
      if(chVal)
      { Inp+=chLen; InpLen-=chLen;
      }
      else
      { if(ID==Telem_Dev_BattLevel) OutLen+=sprintf(Out+OutLen, " Bat=%u%%", Value);
        if(ID==Telem_Dev_BattVolt)  OutLen+=sprintf(Out+OutLen, " Bat=%5.3fV", Float(Value));
        if(ID==Telem_Dev_AirUtil)   OutLen+=sprintf(Out+OutLen, " Air=%4.2f%%", Float(Value));
        if(ID==Telem_Dev_ChanUtil)  OutLen+=sprintf(Out+OutLen, " Chan=%4.2f%%", Float(Value));
        if(ID==Telem_Dev_UpTime)    OutLen+=sprintf(Out+OutLen, " Up=%us", Value);
      }
    }
    return OutLen; }

  static float Float(uint32_t IntVal)
  { return *((float *)(&IntVal)); }

  static const uint8_t Telem_Env_Temperature = 1; // [degC] float
  static const uint8_t Telem_Env_RelHumid    = 2; // [%] float
  static const uint8_t Telem_Env_BaroPress   = 3; // [hPa] float
  static const uint8_t Telem_Env_GasResist   = 4; // [MOhm] float
  static const uint8_t Telem_Env_Voltage     = 5; // [V] float
  static const uint8_t Telem_Env_Current     = 6; // [A] float
  static const uint8_t Telem_Env_IAQ         = 7; // [] float  Indoor Air Quality
  static const uint8_t Telem_Env_RadarDist   = 8; // [mm] float
  static const uint8_t Telem_Env_Lux         = 9; // [lux] float
  static const uint8_t Telem_Env_VISlux      =10; // [] float
  static const uint8_t Telem_Env_IRlux       =11; // [] float
  static const uint8_t Telem_Env_UVlux       =12; // [] float
  static const uint8_t Telem_Env_WindDir     =13; // [deg] uint
  static const uint8_t Telem_Env_WindSpeed   =14; // [m/s] float
  static const uint8_t Telem_Env_Weight      =15; // [kG] float
  static const uint8_t Telem_Env_WindGust    =16; // [m/s] float
  static const uint8_t Telem_Env_WindLull    =17; // [m/s] float

  static int PrintTelem_Env(char *Out, const uint8_t *Inp, int InpLen)
  { int OutLen=sprintf(Out, " Env:");
    uint8_t ID, Wire;
    for( ; InpLen>0; )
    { int Len=ReadKey(ID, Wire, Inp, InpLen); if(Len<=0) return Len;
      Inp+=Len; InpLen-=Len;
      uint32_t Value=0; const uint8_t *chVal=0; uint32_t chLen=0;
            if(isWireInt32 (Wire)) Len=ReadInt32 (Value, Inp, InpLen);
      else  if(isWireVarInt(Wire)) Len=ReadVarInt(Value, Inp, InpLen);
      else  if(isWireBytes (Wire)) { chVal=Inp; Len=ReadVarInt(chLen, Inp, InpLen); }
      else break;
      if(Len<=0) return Len;
      Inp+=Len; InpLen-=Len; if(chVal) chVal+=Len;
      if(chVal)
      { Inp+=chLen; InpLen-=chLen;
      }
      else
      { if(ID==Telem_Env_Temperature) OutLen+=sprintf(Out+OutLen, " Temp=%3.1fdegC", Float(Value));
        if(ID==Telem_Env_RelHumid)    OutLen+=sprintf(Out+OutLen, " RelHum=%3.1f%%", Float(Value));
        if(ID==Telem_Env_BaroPress && Value>0)   OutLen+=sprintf(Out+OutLen, " Baro=%4.2fhPa",  Float(Value));
        if(ID==Telem_Env_GasResist)   OutLen+=sprintf(Out+OutLen, " Gas=%1.0fMOhm",  Float(Value));
        if(ID==Telem_Env_Voltage)     OutLen+=sprintf(Out+OutLen, " Volt=%5.3fV",    Float(Value));
        if(ID==Telem_Env_Current)     OutLen+=sprintf(Out+OutLen, " Curr=%3.1fmA",   Float(Value));
        if(ID==Telem_Env_IAQ)         OutLen+=sprintf(Out+OutLen, " IAQ=%u", Value);
        if(ID==Telem_Env_WindDir)     OutLen+=sprintf(Out+OutLen, " Wind=%udeg", Value);
        if(ID==Telem_Env_WindSpeed)   OutLen+=sprintf(Out+OutLen, " Wind=%3.1fm/s", Float(Value));
        if(ID==Telem_Env_WindGust)    OutLen+=sprintf(Out+OutLen, " Gust=%3.1fm/s", Float(Value));
        if(ID==Telem_Env_WindLull)    OutLen+=sprintf(Out+OutLen, " Lull=%3.1fm/s", Float(Value));
      }
    }
    return OutLen; }

  static int PrintTelem_Air(char *Out, const uint8_t *Data, int Len)
  { int OutLen=sprintf(Out, " Air:");
    return OutLen; }

  static const uint8_t Telem_Pwr_Volt1 = 1; // [V] float
  static const uint8_t Telem_Pwr_Curr1 = 2; // [mA] float
  static const uint8_t Telem_Pwr_Volt2 = 3; // [V] float
  static const uint8_t Telem_Pwr_Curr2 = 4; // [mA] float
  static const uint8_t Telem_Pwr_Volt3 = 5; // [V] float
  static const uint8_t Telem_Pwr_Curr3 = 6; // [mA] float

  static int PrintTelem_Pwr(char *Out, const uint8_t *Inp, int InpLen)
  { int OutLen=sprintf(Out, " Pwr:");
    uint8_t ID, Wire;
    for( ; InpLen>0; )
    { int Len=ReadKey(ID, Wire, Inp, InpLen); if(Len<=0) return Len;
      Inp+=Len; InpLen-=Len;
      uint32_t Value=0; const uint8_t *chVal=0; uint32_t chLen=0;
            if(isWireInt32 (Wire)) Len=ReadInt32 (Value, Inp, InpLen);
      else  if(isWireVarInt(Wire)) Len=ReadVarInt(Value, Inp, InpLen);
      else  if(isWireBytes (Wire)) { chVal=Inp; Len=ReadVarInt(chLen, Inp, InpLen); }
      else break;
      if(Len<=0) return Len;
      Inp+=Len; InpLen-=Len; if(chVal) chVal+=Len;
      if(chVal)
      { Inp+=chLen; InpLen-=chLen;
      }
      else
      { if(ID==Telem_Pwr_Volt1)     OutLen+=sprintf(Out+OutLen, " Volt1=%5.3fV",    Float(Value));
        if(ID==Telem_Pwr_Curr1)     OutLen+=sprintf(Out+OutLen, " Curr1=%3.1fmA",   Float(Value));
        if(ID==Telem_Pwr_Volt2)     OutLen+=sprintf(Out+OutLen, " Volt2=%5.3fV",    Float(Value));
        if(ID==Telem_Pwr_Curr2)     OutLen+=sprintf(Out+OutLen, " Curr2=%3.1fmA",   Float(Value));
        if(ID==Telem_Pwr_Volt3)     OutLen+=sprintf(Out+OutLen, " Volt3=%5.3fV",    Float(Value));
        if(ID==Telem_Pwr_Curr3)     OutLen+=sprintf(Out+OutLen, " Curr3=%3.1fmA",   Float(Value));
      }
    }
    return OutLen; }

  static int PrintTelem_Stat(char *Out, const uint8_t *Data, int Len)
  { int OutLen=sprintf(Out, " Stat:");
    return OutLen; }

  static int PrintTelem_Health(char *Out, const uint8_t *Data, int Len)
  { int OutLen=sprintf(Out, " Health:");
    return OutLen; }


} ;
