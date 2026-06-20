#pragma once

#include <stdlib.h>
#include <string.h>
#include "intmath.h"
#include "ognconv.h"
#include "bitcount.h"
#include "format.h"
#include "crc1021.h"

class FAMP_Packet                                                   //
{ public:

  const static uint8_t Words = 6;                                   // data size, exclude CRC
  const static uint8_t Bytes = 4*Words;                             // data size, exclude CRC

   union
   { uint32_t Word[Words];
      uint8_t Byte[Bytes];
     struct
     { uint32_t Address :24;
       uint8_t  MsgType : 4;    // 2=FAMP
       uint8_t  AddrType: 2;    // 0=Random, 1=ICAO, 2=FLARM, 3=FLARM-Extended
       uint8_t  Urgency : 2;    // 1=High, 2=Pan-Pan, 3=Mayday

       uint16_t ExtAddr: 16;    //
       uint8_t  Reserved: 6;
       uint8_t  Stealth : 1;
       uint8_t  NoTrack : 1;

       uint8_t  Version : 4;    // 3
       uint8_t  MaxVer  : 4;    // 3
                                // encrypted packet part starts here
       uint8_t  Time    : 6;    // shortened UTC time (4 bits) and two bits for the quarter of a second [0.25sec]
       uint8_t  AcftType: 5;
       uint16_t Alt     :13;    // [m] ERC( , 12)
       uint32_t Lat     :20;    //
       uint32_t Lon     :20;    //
       uint16_t Turn    : 9;    // [0.05deg/s] SignERC( , 6, 8);
       uint16_t Speed   :10;    // [0.1m/s]     ERC( , 8);
       uint16_t Climb   : 9;    // [0.1m/s] SignERC( , 6, 8);
       uint16_t Track   :10;    // [0.5deg]
       uint8_t  Move    : 2;    // 1=ground, 2=airborne, 3=circling
       uint8_t  HorAcc  : 6;    // [0.1m]   ERC( , 3);
       uint8_t  VertAcc : 5;    // [0.25m]  ERC( , 2)
       uint8_t  VelAcc  : 5;    // [0.1m/s] ERC( , 3)
       uint8_t  SIL     : 2;    //
       uint8_t  SDA     : 2;    //
       uint8_t  NIC     : 4;    // 11=7.5m, 10=25m, 9=75m
     }  __attribute__((packed));
   } ;

  public:

   void Print(int32_t RefLat, int32_t RefLon) const
   { int32_t Lat = getLatitude(RefLat);
     int32_t Lon = getLongitude(RefLon, Lat);
     printf("%c%c%X:%X:%06X [%+09.5f, %+010.5f]deg %5dm %+5.1fm/s %02dx%02dm m%d",
            NoTrack?'T':' ', Stealth?'S':' ', AcftType, AddrType, Address,
            1e-7*Lat, 1e-7*Lon, getAltitude(),
            0.1*getClimb(), getHorAcc(), getVerAcc(), Move );
     printf(" %05.1fdeg %5.1fm/s %+5.1fdeg/s", 0.5*Track, 0.1*getSpeed(), 0.05*getTurn());
     printf("\n"); }

   int Print(char *Line, int32_t RefLat, int32_t RefLon) const
   { int Len=0;
     int32_t Lat = getLatitude(RefLat);
     int32_t Lon = getLongitude(RefLon, Lat);
     Len+=sprintf(Line+Len, "%c%c%X:%X:%06X [%+09.5f, %+010.5f]deg %5dm %+5.1fm/s %02dx%02dm m%d",
            NoTrack?'T':' ', Stealth?'S':' ', AcftType, AddrType, Address,
            1e-7*Lat, 1e-7*Lon, getAltitude(),
            0.1*getClimb(), getHorAcc(), getVerAcc(), Move );
     Len+=sprintf(Line+Len, " %05.1fdeg %5.1fm/s %+5.1fdeg/s", 0.5*Track, 0.1*getSpeed(), 0.05*getTurn());
     Len+=sprintf(Line+Len, " NIC:%d SIL:%d SDA:%d", NIC, SIL, SDA);
     return Len; }

   void setPosTime(uint32_t Timestamp)
   { Time = (uint8_t)((Timestamp&0xF)<<2); }

   uint32_t getPosTime(int8_t &qSec, uint32_t TimeRef) const
   { qSec = Time&3;                                           // [1/4s]
     int8_t UTCtime = TimeRef&0xF;                            // [sec] shortened UTC time
     int8_t FLRtime = Time>>2;                                // [sec] shortened FLR time
     int8_t Diff = FLRtime-UTCtime;
     if(Diff<(-8)) Diff+=16;
     else if(Diff>7) Diff-=16;
     return TimeRef+Diff; }

      void setLatitude (int32_t NewLat) { Lat=encodeCoord(NewLat, 52); }
      void setLongitude(int32_t NewLon, int32_t NewLat)
      { int32_t OwnLat=getLatitude(NewLat);
        Lon=encodeCoord(NewLon, CoordDiv(OwnLat)); }

   void setPosition(int32_t Lat, int32_t Lon) { setLatitude(Lat); setLongitude(Lon, Lat); }

   static int32_t encodeCoord(int32_t Coord, int32_t Div=52)
   { const int32_t Mod = 0x100000;
     if(Coord>=0) return ((Coord+(Div/2))/Div)%Mod;
     int32_t Enc = ((Coord-(Div/2))/Div)%Mod;
     if(Enc<0) Enc+=Mod;
     return Enc; }

   int32_t getLatitude (int32_t RefLat) const { return decodeCoord(Lat, RefLat, 52); }                               // [1e-7 deg]
   int32_t getLongitude(int32_t RefLon, int32_t OwnLat) const { return decodeCoord(Lon, RefLon, CoordDiv(OwnLat)); } // [1e-7 deg]

   static int32_t decodeCoord(int32_t Coord, int32_t RefCoord, int32_t Div=52)
   { const int32_t Mod  = 0x100000;
     const int32_t Mod2 = 0x080000;
     const int32_t Mask = 0x0FFFFF;
     // const int32_t Shift = 20;
     int32_t Ref  = RefCoord/Div;
     int32_t Diff = (Coord-Ref)&Mask;
     // printf("decodeCoord(%d, %d, %d) Diff:0x%05X\n", Coord, RefCoord, Div, Diff);
     if(Diff>=Mod2) Diff-=Mod;
     return (Ref+Diff)*Div; }

   static int32_t CoordDiv(int32_t Lat)
   { static const int8_t Deg[7] = { 12, 39, 56, 67, 73, 79, 82 };
     if(Lat<0) Lat=(-Lat);
     int8_t LatDeg = Lat/10000000;
     int32_t Div=52;
     for(int Idx=0; Idx<7; Idx++)
     { int32_t Diff=LatDeg-Deg[Idx];
       if(Diff<0) continue;
       int Shift=Idx-1;
       if(Shift>=0) Div += Diff<<  Shift ;
              else  Div += Diff>>(-Shift); }
     // printf("CoordDiv(%d) LatDeg:%d Div:%d\n", Lat, LatDeg, Div);
     return Div; }

   int16_t getAltitude(void) const { return (int16_t)decodeERC(Alt, 12)-1000; }      // [m]
  uint16_t getSpeed   (void) const { return          decodeERC(Speed, 8); }          // [0.1m/s]
   int16_t getClimb   (void) const { return      decodeSignERC(Climb, 6, 8); }       // [0.1m/s]
   int16_t getTurn    (void) const { return      decodeSignERC(Turn , 6, 8); }       // [0.05deg/s]
  uint16_t getHorAcc  (void) const { return          decodeERC(HorAcc , 3); }        // [0.1m]
  uint16_t getVerAcc  (void) const { return          decodeERC(VertAcc, 2); }        // [0.25m]
  uint16_t getVelAcc  (void) const { return          decodeERC(VelAcc , 3); }        // [0.1m/s]

   static uint16_t encodeERC(uint16_t Value, const uint8_t MantBits)
   { uint16_t Top=1; Top<<=MantBits;
     if(Value<Top) return Value;
     uint8_t Exp=0;
     while(Value>=((Top<<(Exp+1))-Top)) Exp++;
     uint16_t Mant = (Value+Top)>>Exp;
     Mant-=Top;
     if(Mant>=Top) Mant = Top-1;
     return (Exp<<MantBits) | Mant; }

   static uint16_t decodeERC(uint16_t Code, const uint8_t MantBits)
   { uint8_t Exp = Code>>MantBits;
     uint16_t Top=1; Top<<=MantBits;
     uint16_t Mant = Code&(Top-1);
     return ((Top+Mant)<<Exp)-Top; }

   static int16_t decodeSignERC(uint16_t Code, const uint8_t MantBits, uint8_t SignBit)
   { uint16_t Sign=1; Sign<<=SignBit;
     if(Code&Sign) return -(int16_t)decodeERC(Code^Sign, MantBits);
              else return           decodeERC(Code,      MantBits); }

   void Decrypt(uint32_t *Nonce, uint32_t Time)
   { const uint32_t FAMPkey[4] = { 0xA5F9B21C, 0xAB3F9D12, 0xC6F34E34, 0xD72FA378 };  // constant key for FAMP packets
     XXTEA_Decrypt(Word+2, 4, FAMPkey, 6);
     Nonce[0] = Word[0];
     Nonce[1] = Word[1];
     Nonce[2] = Time>>4;
     Nonce[3] = 0x956F6C77;
     NONCE_MIX((uint8_t *)Nonce);
     for(int Idx=0; Idx<4; Idx++)
       Word[Idx+2]^=Nonce[Idx];
   }

   void Encrypt(uint32_t *Nonce, uint32_t Time)
   { const uint32_t FAMPkey[4] = { 0xA5F9B21C, 0xAB3F9D12, 0xC6F34E34, 0xD72FA378 };  // constant key for FAMP packets
     Nonce[0] = Word[0];
     Nonce[1] = Word[1];
     Nonce[2] = Time>>4;
     Nonce[3] = 0x956F6C77;
     NONCE_MIX((uint8_t *)Nonce);
     for(int Idx=0; Idx<4; Idx++)
       Word[Idx+2]^=Nonce[Idx];
     XXTEA_Encrypt(Word+2, 4, FAMPkey, 6); }

   static uint32_t NONCE_MX(uint32_t Y, uint32_t Z, uint32_t Sum)
   { return ((((Z>>5) ^ (Y<<2)) + ((Y>>3) ^ (Z<<4))) ^ (Sum^Y)); }

   static void NONCE_MIX(uint8_t *Data, uint8_t Bytes=16, uint8_t Loops=2)
   { const uint32_t Delta = 0x9e3779b9;
     uint32_t Sum = 0;
     uint32_t Z = Data[Bytes-1]; uint32_t Y;
     for( ; Loops; Loops--)
     { Sum += Delta;
       for (uint8_t P=0; P<(Bytes-1); P++)
       { Y = Data[P+1];
         Z = Data[P] += NONCE_MX(Y, Z, Sum); }
       Y = Data[0];
       Z = Data[Bytes-1] += NONCE_MX(Y, Z, Sum); }
   }

}  __attribute__((packed));

