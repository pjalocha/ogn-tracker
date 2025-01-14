#include <stdio.h>
#include <stdint.h>

#include "ogn.h"

// ===========================================================================================

class FlightMonitor
{ public:
   GPS_Position FirstLock;              // first (solid) GPS lock
   GPS_Position Takeoff;                // est. takeoff position
   GPS_Position Landing;                // est. landing position
   GPS_Position MaxAltitude;            // est. positon at peak altitude
   GPS_Position Operator;               // operator position (for drones)
   uint32_t FlownDist;                  // distance flown
   static const uint8_t  MinHold  =  5; // [sec] minimum hold time before takeoff declared
   static const uint16_t MinSpeed = 40; // [0.1m/s] minimum horiontal speed to trigger the takeoff (or vertical speed 1/4 of this limit)
   uint8_t HoldTime;                    // [sec]
   uint8_t FirstInit;                   // [sec]

  public:

   FlightMonitor()
   { }

   void Clear(void)
   { FirstLock.Clear();
     Takeoff.Clear();
     FlownDist=0;
     Landing.Clear();
     MaxAltitude.Clear();
     HoldTime=0;
     FirstInit=0; }

   static char Code36(int Num)       // coding of numbers (like day of the month) in IGC file names
   { if(Num<=0) return '0';
     if(Num<10) return '0'+Num;
     if(Num<36) return 'A'+(Num-10);
     return '_'; }

   // produce short IGC file name (a three-character Serial)
   int ShortName(char *Name, uint8_t TakeoffNum, const char *Serial) const
   { return ShortName(Name, Takeoff, TakeoffNum, Serial); }

   // produce short IGC file name
   static int ShortName(char *Name, const GPS_Position &Takeoff, uint8_t TakeoffNum, const char *Serial)
   { int Len=0;
     Name[Len++]='0'+Takeoff.Year%10;                  // Year (last digit)
     Name[Len++]=Code36(Takeoff.Month);                // encoded month
     Name[Len++]=Code36(Takeoff.Day);                  // encoded day
     Name[Len++]='O';                                  // = OGN
     Len+=Format_String(Name+Len, Serial);             // three-letter serial
     Name[Len++]=Code36(TakeoffNum);                   // flight of the day
     Len+=Format_String(Name+Len, ".IGC");             // extension
     Name[Len]=0;
     // printf("ShortName[%d]: %s\n", Len, Name);
     return Len; }

   // produce long IGC file name
   // int LongName(char *Name, const char *Serial) const
   // { int Len=0;
   //   Name[Len]=0; return 0; }

   // does the GPS position meed the  in-flight criteria ?
   static int FlightThresh(const GPS_Position &Position, uint16_t MinSpeed) // [0.1m/s]
   { if(!Position.isValid()) return -1;                        // if position not valid then give up
     if(Position.Altitude>20000) return 1;                     // [0.1] Altitude above 2000m implies a flight
     uint16_t Speed=Position.Speed;                            // [0.1m/s]  Horizontal speed
     int16_t Climb=Position.ClimbRate;                         // [0.1m/s]  Vertical speed
     uint8_t DOP=Position.PDOP; if(DOP==0) DOP=Position.HDOP;  // [0.1]     Dilution of Precision
     Speed += abs(Climb)*4;                                    // [0.1m/s] take horizontal speed and four times the (absolute) vertical speed
     if(DOP>10) { Speed = (Speed*10)/DOP; }                    // if DOP>1 then divide by DOP
     return Speed>=MinSpeed; }

   bool inFlight(void) const { return Takeoff.isValid() && !Landing.isValid(); }

   int SeekFirstLock(const GPS_Position &Position)
   { if(!Position.isValid()) return 0;                         // GPS must have a fix
     if(!FirstLock.isValid()) { FirstLock=Position; return 1; } // if FirstLock not fixed then copy the GPS position
     FirstInit++;
     if(Position.Speed>10) return 0;                           // if moving then avoid
     if(FirstInit>30) return 0;                                // 
     int8_t Diff = Position.HDOP-FirstLock.HDOP;               // correct FirstFix if better HDOP position withint 30 sec
     if(Diff<0) { FirstLock=Position; return 1; }
     return 0; }

   int Process(const GPS_Position &Position)                   // precess the GPS positions
   { // Position.Print();
     SeekFirstLock(Position);
     if(inFlight())                                            // if already in flight
     { // FlownDist += GPS.Speed;                                 // [0.1m]
       int Det=FlightThresh(Position, MinSpeed/2);             // check in-flight criteria with half the limit
       if(Det<=0)                                              // if fail
       { HoldTime++;                                           // count the holding time
         if(HoldTime>=2*MinHold)                               // if over twice the limit
         { Landing=Position; HoldTime=0;                       // then declare landing, store landing position
           // char Name[16]; ShortName(Name, "XXX");
           // printf("Landing #%d: %s\n", TakeoffCount, Name);
         }
       }
       else HoldTime=0;                                        // if in-flight criteria satisfied then clear the holding time
     }
     else                                                      // if not in flight yet
     { int Det=FlightThresh(Position, MinSpeed);               // check in-flight criteria with normal limits
       if(Det>0)                                               // if criteria satisfied: we switch to airborne !
       { HoldTime++;                                           // count the holding time
         if(HoldTime>=MinHold)                                 // if enough
         { Takeoff=Position; FlownDist=0;                      // declare takeoff, start counting the distance flown
           if(Operator.isValid()) Takeoff=Operator;
           Landing.Clear(); HoldTime=0;                        // clear the landing position
           // char Name[16]; ShortName(Name, "XXX");
           // printf("Takeoff #%d: %s\n", TakeoffCount, Name);
         }
       }
       else
       { Operator=Position;                                   // if takeoff criteria not met, then store operator position
         HoldTime=0; }                                        // clear the holding time and wait
     }
     return inFlight(); }

} ;

// ===========================================================================================
