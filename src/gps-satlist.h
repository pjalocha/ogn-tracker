#pragma once

#include <algorithm>

#include "format.h"
#include "nmea.h"

/* sample GSA and GSV data
$GNGGA,130831.00,5145.95529,N,00111.50572,W,1,11,0.98,87.4,M,47.0,M,,*62
$GNGSA,A,3,08,10,01,23,02,27,,,,,,,1.51,0.98,1.14,1*0F
$GNGSA,A,3,02,15,34,30,,,,,,,,,1.51,0.98,1.14,3*01
$GNGSA,A,3,43,,,,,,,,,,,,1.51,0.98,1.14,4*03
$GNGSA,A,3,,,,,,,,,,,,,1.51,0.98,1.14,5*05
$GPGSV,2,1,07,01,37,258,34,02,69,276,29,08,78,187,19,10,47,059,16,1*60
$GPGSV,2,2,07,23,08,041,15,27,41,136,14,32,23,106,17,1*5B
$GPGSV,2,1,05,03,08,207,,14,19,319,,16,00,174,,22,07,330,,0*69
$GPGSV,2,2,05,22,07,330,,0*57
$GAGSV,2,1,06,02,12,181,09,15,34,258,31,27,37,300,14,30,45,227,27,7*7F
$GAGSV,2,2,06,34,85,328,12,36,36,070,10,7*73
$GAGSV,2,1,05,04,09,022,,05,13,122,,06,18,046,,09,22,070,,0*7A
$GAGSV,2,2,05,09,22,070,,0*4F
$GBGSV,1,1,01,43,62,288,23,1*47
$GBGSV,1,1,01,11,86,018,,0*41
$GQGSV,1,1,00,0*64
*/

class GPS_Sat
{ public:

  static const uint8_t Sys_GQ = 0; // QZSS
  static const uint8_t Sys_GP = 1; // GPS
  static const uint8_t Sys_GL = 2; // GLONASS
  static const uint8_t Sys_GA = 3; // Galileo
  static const uint8_t Sys_GB = 4; // BeiDou

  union
  { uint32_t Word;      // Pack all satellite data into one 32-bit word
    struct
    { uint16_t Azim :6; // [6deg] 0..60, >60 means invalid sky position
      uint8_t  Elev :4; // [6deg] 0..15
      uint8_t   SNR :6; // [dB/Hz]   0 means invalid
      uint8_t  Time :4; // [sec] 0..14, 15 means invalid
      bool      Fix :1; // Included for fix (listed by GSA)
      uint16_t  PRN :8; // [number] GPS converts to range 1..99 thus we might save one bit
      uint16_t  Sys :3; // [sys-id] sat system/constallation
    } __attribute__((packed)) ;
  } ;

  void Clear(void) { Word=0; }

  bool hasSkyPos(void) const { return Azim<=60; } // is Elev:Azim in the sky valid ?
  bool hasSNR(void)    const { return SNR>0; }    // is SNR valid ?
  bool hasTime(void)   const { return Time<15; }  // is Time valid ?

  static const char *SysName(uint8_t Sys)
  { const char *SysTable[8] = { "QZ", "GP", "GL", "GA", "BD", "--", "--", "--" } ;
    return SysTable[Sys]; }

  void Print(void) const
  { printf("%s #%02d", SysName(Sys), PRN);
    if(hasSkyPos()) printf(" %02d:%03ddeg", Elev*6, (uint16_t)Azim*6);
             else   printf(" --:---deg");
    if(hasSNR())    printf(" %2ddB", SNR);
             else   printf(" --dB");
    if(hasTime())   printf(" %02ds", Time);
             else   printf(" --s");
    if(Fix) printf(" +");
    printf("\n"); }

} ;

 class GPS_SatList
{ public:
   static const uint8_t MaxSize = 60;
   GPS_Sat Sat[MaxSize];  // list of satellites
   uint8_t Size;          // number of satellites in the list
   uint8_t qSec;          // [sec]
   uint8_t BurstGSV;
   uint8_t BurstGSA;

   uint8_t FixMode;       // 1=no fix, 2=2-D, 3=3-D
   uint8_t PDOP;          // [0.1]
   uint8_t HDOP;          // [0.1]
   uint8_t VDOP;          // [0.1]

   uint8_t VisSNR [8];    // [0.25dB] average SNR of visible satellites
   uint8_t VisSats[8];    // [count] of visible satellites
   uint8_t FixSNR [8];    // [0.25dB] average SNR of satellites in the fix
   uint8_t FixSats[8];    // [count] of satellites in the fix

  public:
   GPS_SatList() { Clear(); }

   void Clear(void)
   { Size=0; qSec=15;
     BurstGSV=0; BurstGSA=0;
     ClearStats(); }

   uint16_t getSysStatus(uint8_t Sys)
   { uint16_t Stat = FixSats[Sys]; Stat<<=4;
     Stat |= VisSats[Sys]; Stat<<=8;             // upper byte it FixSata (upper nibble) and VisSats (lower nibble)
     Stat |= VisSNR[Sys];                        // lower byte = SNR
     return Stat; }

   void PrintSats(void) const
   { printf("GPS_SatList[%2d] %02ds\n", Size, qSec);
     for(uint8_t Idx=0; Idx<Size; Idx++)
     { printf("%02d: ", Idx); Sat[Idx].Print(); }
   }

   int PrintStats(char *Line) const
   { int Len=sprintf(Line, "SatSNR:");
     for(uint8_t Sys=0; Sys<8; Sys++)
     { if(VisSats[Sys]==0) continue;
       Len+=sprintf(Line+Len, " %s:%d/%4.1f:%d/%4.1fdB",
          GPS_Sat::SysName(Sys), FixSats[Sys], 0.25*FixSNR[Sys], VisSats[Sys], 0.25*VisSNR[Sys] );
     }
     Len+=sprintf(Line+Len, " %d:%3.1f/%3.1f/%3.1f", FixMode, 0.1*PDOP, 0.1*HDOP, 0.1*VDOP);
     return Len; }

   void PrintStats(void) const
   { printf("GPS Stats\n");
     for(uint8_t Sys=0; Sys<8; Sys++)
     { if(VisSats[Sys]==0) continue;
       printf("%s %2d/%4.1fdB %2d/%4.1fdB\n",
          GPS_Sat::SysName(Sys), FixSats[Sys], 0.25*FixSNR[Sys], VisSats[Sys], 0.25*VisSNR[Sys] );
     }
   }

   void ClearStats(void)
   { for(uint8_t Sys=0; Sys<8; Sys++)
     { VisSNR[Sys]=0; VisSats[Sys]=0;
       FixSNR[Sys]=0; FixSats[Sys]=0; }
   }

   uint8_t CalcStats(void)
   { uint8_t SNR; return CalcStats(SNR); }

   uint8_t CalcStats(uint8_t &AverSNR)                   // calc. stats for every GNSS system
   { uint32_t VisSum[8];
     uint32_t FixSum[8];
     for(uint8_t Sys=0; Sys<8; Sys++)                    // clear SNR sums for all systems
     { VisSum[Sys]=0; VisSats[Sys]=0;
       FixSum[Sys]=0; FixSats[Sys]=0; }
     uint8_t TotSat=0;
     uint32_t TotSum=0;
     for(uint8_t Idx=0; Idx<Size; Idx++)                 // loop over satellites
     { if(Sat[Idx].SNR==0) continue;                     // avoid those without SNR
       uint8_t Sys=Sat[Idx].Sys;                         //
       TotSum+=Sat[Idx].SNR;
       VisSum[Sys]+=Sat[Idx].SNR; VisSats[Sys]++;        // add up as a visible satellite
       if(Sat[Idx].Fix)                                  // if this satellite is included in the fix
       { FixSum[Sys]+=Sat[Idx].SNR; FixSats[Sys]++; }    // add up as a fix-included satellite
       TotSat++; }
     for(uint8_t Sys=0; Sys<8; Sys++)
     { if(VisSats[Sys]) VisSNR[Sys]=VisSum[Sys]*4/VisSats[Sys];
                   else VisSNR[Sys]=0;
       if(FixSats[Sys]) FixSNR[Sys]=FixSum[Sys]*4/FixSats[Sys];
                   else FixSNR[Sys]=0;
     }
     if(TotSat) AverSNR = TotSum*4/TotSat;                // return the average sat. SNR
         else   AverSNR = 0;
     return TotSat; }                                     // return number of satellites

   uint8_t Find(uint8_t Sys, uint8_t PRN) const      // find given satellite by Sys and PRN
   { uint8_t Idx=0;
     for( ; Idx<Size; Idx++)
     { if(Sat[Idx].PRN!=PRN) continue;
       if(Sat[Idx].Sys!=Sys) continue;
       break; }
     return Idx; }

   uint8_t Find(uint8_t PRN) const                  // find satellite only by PRN
   { uint8_t Idx=0;
     for( ; Idx<Size; Idx++)
     { if(Sat[Idx].PRN!=PRN) continue;
       break; }
     return Idx; }

   void Delete(uint8_t Idx)
   { if(Idx>=Size) return;
     if(Idx==Size-1) { Size--; return; }
     Size--; Sat[Idx]=Sat[Size]; }

   void CleanFix(uint8_t Time=15)
   { for(uint8_t Idx=0; Idx<Size; Idx++)
     { if(Sat[Idx].Time!=Time) Sat[Idx].Fix=0; }
   }

   void Clean(uint8_t Time)                       //
   { for(uint8_t Idx=0; Idx<Size; )
     { if(Sat[Idx].Time==Time) Delete(Idx);
       else Idx++; }
   }

   uint8_t Add(uint8_t Sys, uint8_t PRN, uint16_t Elev, uint16_t Azim, uint8_t SNR, uint8_t Time)
   { // printf("Add: Sys:%d PRN:%02d Elev:%02d Azim:%03d SNR:%2ddB\n", Sys, PRN, Elev, Azim, SNR);
     uint8_t Idx=Find(Sys, PRN);
     if(Idx>=MaxSize) return Idx;
     GPS_Sat &New = Sat[Idx];
     if(Idx==Size) New.Clear();
     New.Sys=Sys;
     New.PRN=PRN;
     New.Elev=(Elev+3)/6;
     New.Azim=(Azim+3)/6;
     if(New.Time==Time) { if(SNR>0) New.SNR=SNR; }
                   else { New.SNR=SNR; }
     New.Time=Time;
     if(Idx==Size) Size++;
     return Idx; }

   static bool Less(GPS_Sat &Sat1, GPS_Sat &Sat2) { return Sat1.Word<Sat2.Word; }
   static bool LowerSNR(GPS_Sat &Sat1, GPS_Sat &Sat2) { return Sat1.SNR<Sat2.SNR; }
   static bool HigherSNR(GPS_Sat &Sat1, GPS_Sat &Sat2) { return Sat1.SNR>Sat2.SNR; }

   void Sort(void)
   { if(Size<=1) return;
     std::sort(Sat, Sat+Size, Less); }

   int Process(NMEA_RxMsg &NMEA)
   { if(NMEA.isGxGSV()) return ProcessGSV(NMEA);
     if(NMEA.isGxGSA()) return ProcessGSA(NMEA);
     if(NMEA.isGxGGA()) return ProcessGGA(NMEA);
     if(NMEA.isGxRMC()) return ProcessRMC(NMEA);
     return 0; }

   int ProcessRMC(NMEA_RxMsg &RMC)
   { BurstGSV=0;
     BurstGSA=0;
     const char *Time = (const char *)RMC.ParmPtr(0);
     bool NewTime=0;
     if(Time[6]=='.')
     { int8_t Sec=Read_Dec2(Time+4);
       if(Sec>=0) { Sec%=15; NewTime=Sec!=qSec; qSec=Sec; }
     }
     if(NewTime) { Clean(qSec); /* Sort(); CalcStats(); */ }
     // PrintSats(); PrintStats();
     return 0; }

   int ProcessGGA(NMEA_RxMsg &GGA)
   { BurstGSV=0;
     BurstGSA=0;
     const char *Time = (const char *)GGA.ParmPtr(0);
     bool NewTime=0;
     if(Time[6]=='.')
     { int8_t Sec=Read_Dec2(Time+4);
       if(Sec>=0) { Sec%=15; NewTime=Sec!=qSec; qSec=Sec; }
     }
     if(NewTime) { Clean(qSec); /* Sort(); CalcStats(); */ }
     // PrintSats(); PrintStats();
     return 0; }

   int ProcessGSA(NMEA_RxMsg &GSA)
   { BurstGSV=0;
     if(BurstGSA==0) CleanFix();
     BurstGSA++;
     int8_t Sys=(-1);
     if(GSA.Parms<17) return 0;
     if(GSA.Parms>=18) { Sys=Read_Dec1(GSA.ParmPtr(17)[0]); }       // system-id at the 18th parameter
     // if(Sys<0) Sys=GPS_Sat::Sys_GP;                                 // if not readable assume 1 thus GPS
     for(int Par=2; Par<14; Par++)                                  // scan parameters for satellite PRNs
     { int8_t PRN=Read_Dec2((const char *)GSA.ParmPtr(Par)); if(PRN<0) break;  // read PRN, if non, there is no more to read
       uint8_t Idx;
       if(Sys>=0) Idx=Find(Sys, PRN);                               // find this PRN in the satellite list
             else Idx=Find(PRN);
       if(Idx>Size) continue;                                       //
       if(Sys>=0 && Idx==Size && Size<MaxSize)                      // if not found then add this satellite to the list
       { Size++; Sat[Idx].Word=0; Sat[Idx].Sys=Sys; Sat[Idx].PRN=PRN; Sat[Idx].Azim=63; } // with non-valid SNR and sky position
       if(Idx<Size)
       { Sat[Idx].Fix=1; Sat[Idx].Time=qSec; }                        // if found then set time and fix flag
     }
     PDOP = ReadDOP((const char *)GSA.ParmPtr(14));
     HDOP = ReadDOP((const char *)GSA.ParmPtr(15));
     VDOP = ReadDOP((const char *)GSA.ParmPtr(16));
     int8_t Mode=Read_Dec1(GSA.ParmPtr(1)[0]);
     if(Mode>0) FixMode=Mode;
           else FixMode=0;
     return 0; }

   static uint8_t ReadDOP(const char *Inp)
   { int16_t DOP=0;
     if(Read_Float1(DOP, Inp)<=0) return 0;
     if(DOP<0) return 0;
     if(DOP>255) return 255;
     return DOP; }

   int ProcessGSV(NMEA_RxMsg &GSV)
   { BurstGSA=0;
     BurstGSV++;
     uint8_t SatSys=0;                                     // which satelite system
          if(GSV.isGPGSV()) { SatSys=GPS_Sat::Sys_GP; }                  // GPS
     else if(GSV.isGLGSV()) { SatSys=GPS_Sat::Sys_GL; }                  // GLONASS
     else if(GSV.isGAGSV()) { SatSys=GPS_Sat::Sys_GA; }                  // Galileo
     else if(GSV.isBDGSV() || GSV.isGBGSV()) { SatSys=GPS_Sat::Sys_GB; } // Beidou
     else if(GSV.isGQGSV()) { SatSys=GPS_Sat::Sys_GQ; }                  // QZSS
     else return -1;                                       // unknown system: give up
     if(GSV.Parms<3) return -1;
     int8_t Pkts=Read_Dec1((const char *)GSV.ParmPtr(0)); if(Pkts<0) return -1;         // how many messages for this system
     int8_t Pkt =Read_Dec1((const char *)GSV.ParmPtr(1)); if(Pkt <0) return -1;         // which message in the sequence
     int8_t Sats=Read_Dec2((const char *)GSV.ParmPtr(2));                               // total number of satellites
     if(Sats<0) Sats=Read_Dec1((const char *)GSV.ParmPtr(2));                           // could be a single or double digit number
     if(Sats<0) return -1;
     uint8_t Count=0;
     for( int Parm=3; Parm<=GSV.Parms-4; )                                               // up to 4 sats per packet
     { int8_t PRN =Read_Dec2((const char *)GSV.ParmPtr(Parm++)); if(PRN <0) break;      // PRN number
       int8_t Elev=Read_Dec2((const char *)GSV.ParmPtr(Parm++)); // if(Elev<0) break;      // [deg] eleveation
      int16_t Azim=Read_Dec3((const char *)GSV.ParmPtr(Parm++)); // if(Azim<0) break;      // [deg] azimuth
       int8_t SNR =Read_Dec2((const char *)GSV.ParmPtr(Parm++)); if(SNR<0) SNR=0;       // [dB] SNR or absent when not tracked
       if( Elev<0 || Azim<0 ) { Elev=0; Azim=378; }                                     // invalid sky position
       if(SNR<0) SNR=0; else if(SNR>63) SNR=63;
       Add(SatSys, PRN, Elev, Azim, SNR, qSec);
       Count++; }
     return Count; }

} ;

