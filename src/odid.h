#pragma once

#include <stdint.h>
#include <stdio.h>
#include <stdarg.h>
#include <math.h>
#include <string.h>

#include <map>

#include "ogn1.h"
#include "adsl.h"

class FRID_Packet            // French Drone-ID packet content (not the packet structure)
{ public:
   union
   { uint32_t Flags;
     struct
     { uint8_t  Version:8;   // #1  = 0x01
       uint8_t IdentLen:6;
       bool hasVersion:1;    // flags about absence or presense of various elements
       bool identPSN:1;
       bool hasLat:1;        //
       bool hasLon:1;
       bool hasAlt:1;
       bool hasHeight:1;
       bool hasOperLat:1;
       bool hasOperLon:1;
       bool hasSpeed:1;
       bool hasTrack:1;
     } __attribute__((__packed__));
   } ;
   char    Ident[32]; // #2  [up to 30 chars] or #3 ANSI/CTA/2063
   int32_t Lat;       // #4  [1e-5 deg]
   int32_t Lon;       // #5  [1e-5 deg]
   int16_t Alt;       // #6  [m] AMSL
   int16_t Height;    // #7  [m] above takeoff
   int32_t OperLat;   // #8  [1e-5 deg]
   int32_t OperLon;   // #9  [1e-5 deg]
  uint16_t Track;     // #11 [deg]
   uint8_t Speed;     // #10 [m/s]
   uint8_t Spare;

  public:
   void Clear(void) { Flags=0; }

   int Read(const uint8_t *Data)
   { const uint8_t Sign[3] = { 0x6a, 0x5c, 0x35 };
     if(Data[0]!=0xDD) return 0;
     uint8_t Len=Data[1];
     if(Len<3 || memcmp(Data+2, Sign, 3)!=0) return 0;
     int Ret=Read(Data+5, Len-3);
     if(Ret!=(Len-3)) return 0;
     return Ret+5; }

   int Read(const uint8_t *Data, uint8_t Len)
   { Clear();
     uint8_t Idx=0;
     for( ; Idx<Len; )
     { if(Len-Idx<2) return 0;
       uint8_t ElemLen=Data[Idx+1];
       if(ElemLen==0 || ElemLen>32 || ElemLen>Len-Idx-2) return 0;
       uint8_t Ret=readElem(Data+Idx); if(Ret<2 || Ret>Len-Idx) return 0;
       Idx+=Ret; }
     return Idx; }

   uint8_t readElem(const uint8_t *Data)                         // read an element of the French Drone-ID packet
   { uint8_t Type=Data[0];                                       // element-type
     uint8_t Len =Data[1]; if(Len==0 || Len>32) return 0;        // element-length, not including the Type/Len bytes
     switch(Type)                                                // depending on the element-type: read the corresponding piece of data
     { case  1: if(Len!=1) return 0; Version=Data[2]; hasVersion=1; break;
       case  2: readIdent(Data+2, Len); identPSN=0; break;
       case  3: readIdent(Data+2, Len); identPSN=1; break;
       case  4: if(Len!=4) return 0; Lat=readInt32(Data+2);     hasLat=1; break;
       case  5: if(Len!=4) return 0; Lon=readInt32(Data+2);     hasLon=1; break;
       case  6: if(Len!=2) return 0; Alt=readInt16(Data+2);    hasAlt=1; break;
       case  7: if(Len!=2) return 0; Height=readInt16(Data+2); hasHeight=1; break;
       case  8: if(Len!=4) return 0; OperLat=readInt32(Data+2); hasOperLat=1; break;
       case  9: if(Len!=4) return 0; OperLon=readInt32(Data+2); hasOperLon=1; break;
       case 10: if(Len!=1) return 0; Speed=Data[2];             hasSpeed=1; break;
       case 11: if(Len!=2) return 0; Track=readInt16(Data+2);   hasTrack=1; break;
       default: break; }
     return Len+2; }                                             // return the total number of processed bytes

   // read integers which are MSB-first
   static uint16_t readInt16(const uint8_t *Data) { uint16_t Val=Data[0]; Val<<=8; Val|=Data[1]; return Val; }
   static uint32_t readInt32(const uint8_t *Data) { uint32_t Val=readInt16(Data); Val<<=16; Val|=readInt16(Data+2); return Val; }

   void readIdent(const uint8_t *Data, uint8_t Len)
   { uint8_t Idx=0;
     for( ; Idx<Len; Idx++)
     { if(Idx>=32) break;
       Ident[Idx]=Data[Idx]; }
     IdentLen=Idx;
     for( ; Idx<32; Idx++)
       Ident[Idx]=0x00;
   }

   int Write(ADSL_Packet &Packet, uint64_t MAC, int16_t GeoidSepar, uint8_t AcftType=0xD) // convert to an ADS-L packet
   { if((hasVersion && Version!=1) || !hasLat || !hasLon || !hasAlt) return 0;
     if(Lat<(-9000000) || Lat>9000000 || Lon<(-18000000) || Lon>18000000) return 0;
     if(Lat==0 && Lon==0) return 0;                        // unknown position

     Packet.Init();
     Packet.setAddress(MAC&0xFFFFFF);                     // use the transmitter MAC as its 24-bit random ID
     Packet.setAddrTable(4);
     Packet.setAcftTypeOGN(AcftType);
     Packet.setLatUBX(Lat*100);                           // 1e-5 deg -> 1e-7 deg -> ADS-L coordinates
     Packet.setLonUBX(Lon*100);
     Packet.setAlt((int32_t)Alt+GeoidSepar);              // FRID altitude is AMSL; ADS-L altitude is HAE
     Packet.TimeStamp=60;                                 // FRID carries no position time

     if(hasTrack && Track<=359)
       Packet.setTrack((Track*0x200+180)/360);             // degrees -> ADS-L 9-bit circle
     if(hasSpeed)
       Packet.setSpeed((uint16_t)Speed*4);                 // [m/s] -> [0.25 m/s]
     else
       Packet.clrSpeed();
     Packet.clrClimb();
     return 1; }

   int Write(OGN1_Packet &Packet, uint64_t MAC, uint8_t AcftType=0xD) // convert to an OGN packet
   { if((hasVersion && Version!=1) || !hasLat || !hasLon || !hasAlt) return 0;
     if(Lat<(-9000000) || Lat>9000000 || Lon<(-18000000) || Lon>18000000) return 0;
     if(Lat==0 && Lon==0) return 0;                         // unknown position

     Packet.Clear();
     Packet.Header.Address=MAC&0xFFFFFF;                    // use the transmitter MAC as its 24-bit random ID
     Packet.Header.AddrType=0;
     Packet.calcAddrParity();
     Packet.Position.AcftType=AcftType;
     Packet.EncodeLatitude((int32_t)((int64_t)Lat*6));       // 1e-5 deg -> 1/600000 deg
     Packet.EncodeLongitude((int32_t)((int64_t)Lon*6));
     Packet.EncodeAltitude(Alt);                            // FRID altitude is AMSL in meters
     Packet.Position.Time=63;                               // FRID carries no UTC position time
     Packet.EncodeHeading(hasTrack && Track<=359 ? (int16_t)(Track*10) : 0);
     Packet.EncodeSpeed(hasSpeed ? (int16_t)Speed*10 : 0);  // m/s -> 0.1 m/s
     Packet.clrClimbRate();
     Packet.Position.FixMode=1;
     Packet.Position.FixQuality=1;
     Packet.EncodeDOP(0xFF);                                // no position accuracy in FRID
     Packet.clrTurnRate();
     Packet.clrBaro();
     return 1; }

} __attribute__((__packed__));


class ODID_Packet                             // Open-Drone-ID packet (25 bytes)
{ static int append(char *Line, size_t Size, int Len, const char *Format, ...)
  { if(!Line || Size==0) return -1;
    if(Len<0 || (size_t)Len>=Size) return Len;
    va_list Args;
    va_start(Args, Format);
    int Added=vsnprintf(Line+Len, Size-(size_t)Len, Format, Args);
    va_end(Args);
    return Added<0 ? Added : Len+Added; }

  public:

  static const int PktTypes = 6;              // 6 packets types being used

  static const uint8_t PktType_BasicID  = 0;  // basic identification and aircraft-type: obligatory
  static const uint8_t PktType_Position = 1;  // GPS/Baro position: obligatory
  static const uint8_t PktType_Auth     = 2;  // Authentication: optional
  static const uint8_t PktType_SelfID   = 3;  // "Free" ID or emergency message
  static const uint8_t PktType_System   = 4;  // other drones organized with this one, operator/pilot position
  static const uint8_t PktType_OperID   = 5;  // operator/pilot ID

  union
  { uint8_t Header;                          // when 0xFF then this packet is not filled in with actual data
    struct
    { uint8_t Version :4;                    // = 2 (for now always version #2)
      uint8_t Type    :4;                    // = 0..5
    } __attribute__((__packed__));
  } ;

  union
  { uint8_t Data[24];
    struct
    { uint8_t UAType  :4;  // 1=aeroplane, 2=heli/rotor, 3=gyro, 4=hybrid, 5=ormi, 6=glider, 7=kite, 8=balloon, 9=capt. baloon, 10=airship, 11=para, 12=rocket, 13=tethered, 14=obstacle, 15=other
      uint8_t IDtype  :4;  // 1=serial, 2=CAA, 3=UTM
      char    Name[20];
      uint8_t Reserved[3];
    } __attribute__((__packed__)) BasicID;  // Type=0 (obligatory)
    struct
    { uint8_t Page:4;
      uint8_t Type:4;
      uint8_t LastPage;
      uint8_t Length;
      uint32_t TimeStamp;
      uint8_t Data[17];
    } __attribute__((__packed__)) Auth0;    // Type=2
    struct
    { uint8_t Page:4;
      uint8_t Type:4;
      uint8_t Data[23];
    } __attribute__((__packed__)) Auth;    // Type=2
    struct
    { uint8_t IDtype;      // 0=Text Description, 1=Emergency Description
      char    Name[23];
    } __attribute__((__packed__)) SelfID;  // Type=3
    struct
    { uint8_t IDtype;      // 0=OperatorID
      char    Name[23];    // CAA issued operator ID
    } __attribute__((__packed__)) OperID;  // Type=5
    struct
    { union
      { uint8_t Flags;
        struct
        { uint8_t OperLoc  :2;      // Operator Location: 0=Take-Off, 1=Dynamic, 2=Fixed
          uint8_t ClassType:3;      // Classification Type: 0=Undeclared, 1=EU, ... reserved
          uint8_t FlagsRes :3;      // Reserved
        } ;
      } ;
       int32_t OperLat;            // [1e-7deg]
       int32_t OperLon;            // [1e-7deg]
      uint16_t AreaCount;          // number of aircrafts in the area
      uint8_t  AreaRadius;         // [100m]
      uint16_t AreaCeiling;        // [0.5m HAE]
      uint16_t AreaFloor;          // [0.5m HAE]
      union
      { uint8_t UA;                // when ClassType=1
        struct
        { uint8_t Class :4;
          uint8_t Cat   :4;
        } ;
      } ;
      uint16_t OperAlt;            // [0.5m HAE]
      uint32_t OperTime;           // [s] since 01/01/2019  Unix-1546300800
      uint8_t  OperReserv;
    } __attribute__((__packed__)) System; // Type=4
    struct
    { union
      { uint8_t Flags;
        struct
        { bool SpeedMult :1;      // 0.25 or 0.75m/s speed encoding
          bool WestTrack :1;      // 1 if Track>=180 thus the MSB of Track
          uint8_t HeightType:2;   // 0=above-takeoff, 1=AGL
          uint8_t Status :4;      // 0=undeclared, 1=Ground, 2=Airborne, 3=Emergency, 4=Remote-ID-System failure
        } ;
      } ;
      uint8_t  Track;             // [deg]
      uint8_t  Speed;             // [0.25m/s] or [0.75m/s]
       int8_t  Climb;             // [0.5 m/s] - what is the "not-specified" value ?
       int32_t Lat;               // [1e-7deg]
       int32_t Lon;               // [1e-7deg]
      uint16_t AltBaro;           // [0.5m] biased by 1000m pressure altitude
      uint16_t AltGeo;            // [0.5m] biased by 1000m HAE
      uint16_t Height;            // [0.5m] biased by 1000m above takeoff
      uint8_t HorAcc:4;           // NACp  horizontal accuracy:  12=1m, 11=3m, 10=10m, 9=30m, 8=93m, 7=185m, 6=556m, 5=926m, 4=1852m, ...
      uint8_t VertAcc:4;          // GVA   vertical accuracy: 6=1m, 5=3m, 4=10m, 3=25m, 2=45m, 1=150m
      uint8_t SpeedAcc:4;         // NACv  speed accuracy: 4=0.3m/s, 3=1m/s, 2=3m/s, 1=10m/s,
      uint8_t BaroAcc:4;          //
      uint16_t TimeStamp;         // [0.1sec] since full hour
      uint8_t TSAcc:4;            // [0.1sec] Time-Stamp Accuracy
      uint8_t Reserv4bit:4;       //
      uint8_t ReservByte;
    } __attribute__((__packed__)) Position; // Type=1 (obligatory)

  } ;

  public:
   void setInvalid(void) { Header=0xFF; }
   bool isValid(void) const { return Type<=5 && Version==2; }

   void Clean(uint8_t Type, uint8_t Version=2)
   { this->Type=Type; this->Version=Version;
     for(uint8_t Idx=0; Idx<24; Idx++)
       Data[Idx]=0; }

   void setBasicID(uint8_t Type, uint8_t UAType, const char *Serial=0)
   { Clean(PktType_BasicID);
     BasicID.IDtype=Type;
     BasicID.UAType=UAType;
     if(Serial) strncpy(BasicID.Name, Serial, 20); }

   void setOperID(uint8_t Type, const char *Text)
   { Clean(PktType_OperID);
     OperID.IDtype=Type;
     strncpy(OperID.Name, Text, 23); }

   void setSelfID(uint8_t Type, const char *Text)
   { Clean(PktType_SelfID);
     SelfID.IDtype=Type;
     strncpy(SelfID.Name, Text, 23); }

   float getAlt(uint16_t Alt) const { return 0.5f*Alt-1000.0f; }          // [m]
   float getAltBaro(void) const { return getAlt(Position.AltBaro); }      // [m] pressure altitude
   float getAltGeo(void)  const { return getAlt(Position.AltGeo); }       // [m]
   float getHeight(void)  const { return getAlt(Position.Height); }       // [m]

   uint16_t getTrack(void) const                                          // [deg]
   { uint16_t Track=Position.Track; if(Position.WestTrack) Track+=180;    // highest Track bit is elsewhere
     return Track; }
   float getSpeed(void) const                                             // [m/s]
   { if(!Position.SpeedMult) return 0.25f*Position.Speed;
     return 0.75*Position.Speed+63.75f; }
   float getClimb(void) const { return 0.5f*Position.Climb; }             // [m/s]

   uint32_t getTime(int16_t &msTime, uint32_t RefTime, int FwdMargin=10) const // works out the full timestamp from the shorten time
   { msTime=100*(Position.TimeStamp%10);
     int Sec=RefTime%3600;
     int DiffSec=(Position.TimeStamp/10)-Sec;
     if(DiffSec>FwdMargin) DiffSec-=3600;
     else if(DiffSec<=(-3600+FwdMargin)) DiffSec+=3600;
     return RefTime+DiffSec; }           // [sec] return full UTC timestamp and [ms] msTime

   uint8_t getHorAcc(void) const  // return horizontal accuracy [m]
   { uint8_t Acc=Position.HorAcc;
     if(Acc>12) return 0;
     if(Acc< 9) return 63;
     const uint8_t Table[4] = { 30, 10, 3, 1 };
     return Table[Acc-9]; }

   uint8_t getVertAcc(void) const  // return vertical accuracy [m]
   { uint8_t Acc=Position.VertAcc;
     if(Acc>6) return 0;
     if(Acc<2) return 63;
     const uint8_t Table[5] = { 45, 25, 10, 3, 1 };
     return Table[Acc-2]; }

   // encode the position into an ADS-L packet, so we can do a relay
   int Write(ADSL_Packet &Packet, uint64_t MAC, uint8_t AcftType=0xD) // convert to an ADS-L packet
   { if(!isValid() || Type!=PktType_Position) return 0;    // only convert valid position packets
     if(Position.Lat<(-900000000) || Position.Lat>900000000) return 0;
     if(Position.Lon<(-1800000000) || Position.Lon>1800000000) return 0;
     if(Position.Lat==0 && Position.Lon==0) return 0;      // OpenDroneID uses 0/0 for unknown position
     if(Position.AltGeo==0) return 0;                      // zero denotes unavailable HAE altitude

     Packet.Init();
     Packet.setAddress(MAC&0xFFFFFF);                     // use the transmitter MAC as its 24-bit random ID
     Packet.setAddrTable(4);
     Packet.setAcftTypeOGN(AcftType);

     // ADS-L altitude is HAE in whole meters; RemoteID uses half-meters with a 1000m bias.
     int32_t Alt=(int32_t)Position.AltGeo-2000;
     Alt=(Alt>=0 ? (Alt+1)/2 : (Alt-1)/2);                 // round to nearest whole meter
     Packet.setAlt(Alt);
     Packet.setLatUBX(Position.Lat);                      // [1e-7 deg] -> ADS-L FANET coordinates
     Packet.setLonUBX(Position.Lon);

     // ADS-L time is quarter-seconds within a 15-second cycle. RemoteID time
     // is in tenths of a second since the hour; 0xFFFF/out-of-range is unavailable.
     if(Position.TimeStamp<36000)
     { uint16_t Time=(Position.TimeStamp%150)*4;
       Time=(Time+5)/10;
       Packet.TimeStamp=Time%60; }
     else
       Packet.TimeStamp=60;

     if(Position.Track<=179)
     { uint16_t Track=getTrack();
       Packet.setTrack((Track*0x200+180)/360); }           // degrees -> ADS-L 9-bit circle
     if(Position.Speed!=0xFF)
     { uint16_t Speed=Position.SpeedMult ? (3*Position.Speed+255) : Position.Speed;
       Packet.setSpeed(Speed); }                          // native speed encoding -> [0.25 m/s]
     else
       Packet.clrSpeed();
     if(Position.Climb!=(-128))
       Packet.setClimb((int16_t)Position.Climb*4);         // [0.5 m/s] -> [0.125 m/s]
     else
       Packet.clrClimb();

     if(Position.Status==1) Packet.FlightState=1;           // ground
     else if(Position.Status==2) Packet.FlightState=2;      // airborne

     uint8_t HorAcc=getHorAcc();
     if(HorAcc) Packet.setHorAcc(HorAcc);
     uint8_t VerAcc=getVertAcc();
     if(VerAcc) Packet.setVerAcc(VerAcc);
     if(Position.SpeedAcc>=1 && Position.SpeedAcc<=4)
       Packet.VelAccuracy=(Position.SpeedAcc>=3) ? 3 : Position.SpeedAcc;
     return 1; }

   // encode the position into an OGN packet, so we can do a relay
   int Write(OGN1_Packet &Packet, uint64_t MAC, int16_t GeoidSepar, uint8_t AcftType=0xD) // convert to an OGN packet
   { if(!isValid() || Type!=PktType_Position) return 0;    // only convert valid position packets
     if(Position.Lat<(-900000000) || Position.Lat>900000000) return 0;
     if(Position.Lon<(-1800000000) || Position.Lon>1800000000) return 0;
     if(Position.Lat==0 && Position.Lon==0) return 0;      // OpenDroneID uses 0/0 for unknown position
     if(Position.AltGeo==0) return 0;                      // HAE is required for OGN altitude

     // RemoteID altitude is HAE, in half-meters with a 1000m bias. OGN
     // altitude is integer meters AMSL, so remove the bias and geoid offset.
     int32_t Alt=(int32_t)Position.AltGeo-2000;
     Alt=(Alt>=0 ? (Alt+1)/2 : (Alt-1)/2);                 // nearest whole meter
     Alt-=GeoidSepar;
     if(Alt<0) Alt=0;

     Packet.Clear();
     Packet.Header.Address = MAC&0xFFFFFF;                 // take lower 24 bits of the MAC
     Packet.Header.AddrType = 0;                           // we assign random ID assuming the MAC in RemoteID is random
     Packet.calcAddrParity();
     Packet.Position.AcftType = AcftType;
     Packet.EncodeAltitude(Alt);                         // [m]
     Packet.EncodeLatitude((int64_t)Position.Lat*3/50);   // 1e-7 deg -> 1/60000 deg
     Packet.EncodeLongitude((int64_t)Position.Lon*3/50);

     // OGN time is seconds within a minute; 63 means unavailable and makes
     // LookOut fall back to the receive time.
     Packet.Position.Time=(Position.TimeStamp<36000) ? (Position.TimeStamp/10)%60 : 63;
     uint16_t Track=(Position.Track<=179) ? getTrack() : 360; // 0xFF is unavailable
     Packet.EncodeHeading((Track<=359) ? (int16_t)(Track*10) : 0); // [deg] -> [0.1 deg]
     float Speed=(Position.Speed==0xFF) ? 0.0f : getSpeed(); // 0xFF means unavailable
     Packet.EncodeSpeed((int16_t)(Speed*10.0f+0.5f));     // [m/s] -> [0.1 m/s]
     if(Position.Climb!=(-128))
       Packet.EncodeClimbRate((int16_t)Position.Climb*5);  // [0.5 m/s] -> [0.1 m/s]
     else
       Packet.clrClimbRate();

     Packet.Position.FixMode=1;
     Packet.Position.FixQuality=1;
     uint8_t HorAcc=getHorAcc();
     if(HorAcc)
     { int16_t Error=2*HorAcc;                          // OGN relative-position error is in half-meters
       int16_t DOP=(5*Error-2)/2;                       // inverse of Acft_RelPos::Read's DOP mapping
       if(DOP<0) DOP=0;
       if(DOP>255) DOP=255;
       Packet.EncodeDOP((uint8_t)DOP); }
     else
       Packet.EncodeDOP(0xFF);                          // unknown accuracy
     Packet.clrTurnRate();
     int32_t StdAlt = Position.AltBaro;
     if(StdAlt>0)
     { StdAlt-=2000;
       StdAlt=(StdAlt>=0 ? (StdAlt+1)/2 : (StdAlt-1)/2); // nearest whole meter
       Packet.EncodeStdAltitude(StdAlt); }
     else
       Packet.clrBaro();
     return 1; }

   const char *getPktTypeName(void) const
   { const char *Name[16] = { "Basic-ID", "Pos", "Auth", "Self-ID", "System", "Oper-ID", 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 } ;
     return Name[Type]; }

   int Print(char *Line, size_t Size) const
   { if(!Line || Size==0) return -1;
     Line[0]=0;
     int Len=append(Line, Size, 0, "%Xv%X  ", Type, Version);
     if(Type==PktType_BasicID)                    // Basic-ID: ID-type, UAV-type
       Len=append(Line, Size, Len, "BasicID[%X:%X]:%-20.20s", BasicID.IDtype, BasicID.UAType, BasicID.Name);
     else if(Type==PktType_Position)               // Position/Speed/...
     { Len=append(Line, Size, Len, "%04.1fs [%+09.5f,%+010.5f]",
                  0.1*(Position.TimeStamp%600), 1e-7*Position.Lat, 1e-7*Position.Lon);
       if(Position.AltBaro) Len=append(Line, Size, Len, " %3.1f", getAlt(Position.AltBaro));
       else Len=append(Line, Size, Len, " -.-");
       if(Position.AltGeo) Len=append(Line, Size, Len, "/%3.1f", getAlt(Position.AltGeo));
       else Len=append(Line, Size, Len, "/-.-");
       if(Position.Height) Len=append(Line, Size, Len, "/%3.1f", getAlt(Position.Height));
       else Len=append(Line, Size, Len, "/-.-");
       Len=append(Line, Size, Len, "m %03ddeg %3.1fm/s %+4.1fm/s", getTrack(), getSpeed(), getClimb()); }
     else if(Type==PktType_System)                 // System (operator)
     { Len=append(Line, Size, Len, "Oper: [%+09.5f,%+010.5f]", 1e-7*System.OperLat, 1e-7*System.OperLon);
       if(System.OperAlt) Len=append(Line, Size, Len, " %3.1fm", getAlt(System.OperAlt));
       else Len=append(Line, Size, Len, " -.-m"); }
     else if(Type==PktType_SelfID)                // Self-ID
       Len=append(Line, Size, Len, "SelfID[%02X]:%-23.23s", SelfID.IDtype, SelfID.Name);
     else if(Type==PktType_Auth)                  // Authentication
       Len=append(Line, Size, Len, "Auth[%X:%X]", Auth.Type, Auth.Page);
     else if(Type==PktType_OperID)                // Operator-ID
       Len=append(Line, Size, Len, "OperID[%02X]:%-23.23s", OperID.IDtype, OperID.Name);
     return Len; }

} __attribute__((__packed__)) ;


class ODID_Acft
{ public:
   static const int PktTypes = 6;
   uint8_t     PktSeq[PktTypes];      // keeps current sequence number for packet types
   uint32_t  DupCount[PktTypes];
   ODID_Packet Packet[PktTypes];
   uint64_t   MAC;
   uint32_t   UpdTime[PktTypes];
   static const uint8_t MaxChans = 8;
   float      RxRSSI[MaxChans];
   uint32_t   RxRSSItime[MaxChans];
   // float      MaxRxRSSI;
   // uint8_t    MaxRxRSSI;
   char SSID[42];

  public:
   ODID_Acft()
   { MAC=0;
     for(uint8_t Chan=0; Chan<MaxChans; Chan++)
     { RxRSSI[Chan]=0; RxRSSItime[Chan]=0; }
     for(int Idx=0; Idx<PktTypes; Idx++)
       UpdTime[Idx]=0;
     SSID[0]=0; }

   void CleanRxRSSI(uint32_t Time)
   { for(uint8_t Chan=0; Chan<MaxChans; Chan++)
     { uint32_t Diff=Time-RxRSSItime[Chan]; if(Diff<10000) continue;
       RxRSSI[Chan]=0; }
   }

   uint32_t getUpdTime(void) const
   { uint32_t MaxTime=0;
     for(int Idx=0; Idx<PktTypes; Idx++)
     { if(UpdTime[Idx]>MaxTime) MaxTime=UpdTime[Idx]; }
     return MaxTime; }

   int Process(const ODID_Packet &NewPacket, uint8_t Seq, uint32_t Time, int8_t RSSI, uint8_t Chan=0)
   { uint8_t PktType=NewPacket.Type;
     if(PktType>=PktTypes) return 0;
     if(PktSeq[PktType]==Seq) { DupCount[PktType]++; return 0; }
     PktSeq[PktType]=Seq;
     Packet[PktType]=NewPacket;
     UpdTime[PktType]=Time;
     if(RSSI && Chan<MaxChans)
     { if(RxRSSI[Chan]==0) RxRSSI[Chan]  = RSSI;
                      else RxRSSI[Chan] += 0.1f*((float)RSSI-RxRSSI[Chan]);
       RxRSSItime[Chan]=Time; }
     return PktType==1; }                // return 1 for position packets

} ;

class ODID_AcftList
{ public:
   std::map<uint64_t, ODID_Acft> List;
   // static const int MaxIdleTime = 60;
   uint32_t RxID;
    int32_t RxLat, RxLon, RxAlt;
   static const uint32_t MaxIdleTime = 60000;
   static const uint32_t MinCleanTime = 1000;
   uint32_t LastCleanTime;

  public:
   ODID_AcftList() { RxID=0; RxLat=0; RxLon=0; RxAlt=0; LastCleanTime=0; }

   int Acfts(void) const { return List.size(); }

   ODID_Acft *Process(uint64_t MAC, uint8_t PktSeq, const ODID_Packet &Packet, uint32_t Time, int8_t RSSI, uint8_t Chan=0)
   { ODID_Acft &Acft = List[MAC];
     if(Acft.MAC==0) Acft.MAC=MAC;
     int Ret=Acft.Process(Packet, PktSeq, Time, RSSI, Chan); if(Ret<=0) return 0;
     return &Acft; }

   ODID_Acft *Process(uint64_t MAC, uint8_t PktSeq, const uint8_t *Pack, uint8_t PackLen, uint32_t Time, int8_t RSSI, uint8_t Chan=0, const char *SSID=0)
   { ODID_Acft &Acft = List[MAC];
     if(Acft.MAC==0) Acft.MAC=MAC;
     if(SSID && SSID[0]) strcpy(Acft.SSID, SSID);
     uint8_t Type = Pack[0]>>4;
     bool hasPos=0;
     if(Type==0xF)
     { uint8_t Size = Pack[1];
       uint8_t Pages = Pack[2];
       if(Size==25)
       { for(uint8_t Idx=0; Idx<Pages; Idx++)
         { const ODID_Packet *Packet = (const ODID_Packet *)(Pack+3+Idx*Size);
           int Ret = Acft.Process(*Packet, PktSeq, Time, RSSI, Chan);
           if(Ret>0) hasPos=1; }
       }
     }
     else
     { const ODID_Packet *Packet = (const ODID_Packet *)(Pack+5);
       int Ret=Acft.Process(*Packet, PktSeq, Time, RSSI, Chan);
       if(Ret>0) hasPos=1; }
     return hasPos?&Acft:0; }

   int Clean(uint32_t Time)
   { uint32_t Diff = Time-LastCleanTime;
     if(Diff<MinCleanTime) return 0;
     LastCleanTime=Time;
     int Count=0;
     for(std::map<uint64_t, ODID_Acft>::iterator it=List.begin(); it!=List.end(); )
     { ODID_Acft &Acft = it->second;
       Acft.CleanRxRSSI(Time);
       uint32_t Idle = Time - Acft.getUpdTime();
       if(Idle<=MaxIdleTime) { ++it; continue; }
       // printf("Removing Acft %06X from map\n", Acft.Addr);
       it = List.erase(it); Count++; }
     return Count; }

} ;
