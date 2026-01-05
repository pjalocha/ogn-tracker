#include <stdio.h>
#include <stdint.h>

#include "nmea.h"
#include "ogn.h"
#include "flight.h"

// ===========================================================================================

FlightMonitor Flight;

char Line[128];
NMEA_RxMsg NMEA;
GPS_Position Position;
GPS_Position PrevPosition;

int ProcessFile(const char *FileName)
{ FILE *File = fopen(FileName, "rt"); if(File==0) return -1;
  FILE *IGC = 0;
  int TakeoffNum = 0;

  int Err=0;
  NMEA.Clear();
  printf("\n");
  Position.Clear();
  for( ; ; )
  { if(fgets(Line, 128, File)==0) break;
    char *Msg = strchr(Line, '$'); if(Msg==0) continue;
    char *NL = strchr(Line, '\n'); // if(NL==0) continue;
    char *CR = strchr(Line, '\r'); // if(CR==0) continue;
    if(NL==0 && CR==0) continue;
    if(CR) *CR=0;
    if(NL) *NL=0;
    // printf("%s\n", Msg);
     for(const char *Idx=Msg; *Idx; Idx++)
    { NMEA.ProcessByte(*Idx); }
    NMEA.ProcessByte('\n');
    if(NMEA.isComplete())
    { Err=Position.ReadNMEA(NMEA); // printf("Position.ReadNMEA()=> %d\n", Err);
      NMEA.Clear();
      if(Err>0 && Position.hasGPS)
      { // Position.PrintLine(Line);
        // printf("%s", Line);
        Position.calcDifferentials(PrevPosition);
        Position.PrintLine();
        int inFlight = Flight.Process(Position);
        if(inFlight>0)
        { if(IGC==0) { TakeoffNum+=1; Flight.ShortName(Line, TakeoffNum, "XXX"); IGC=fopen(Line, "wt"); printf("IGC: %s\n", Line); }
          if(IGC) { Position.WriteIGC(Line); fprintf(IGC, "%s", Line); }
        }
        else
        { if(IGC) { fclose(IGC); IGC=0; }
        }
        Position.WriteIGC(Line); // printf("IGC: %s\n", Line);
        // Position.WriteAPRS(Line, "TEST", "/g", 0x07123456); printf("APRS: %s\n", Line);
        PrevPosition = Position;
        Position.Clear(); }
    }
    // Position.PrintLine(Line);
    // printf("%s", Line);
    // Position.Print();
    // Err=Position.ReadNMEA(Msg);
    // printf("Position.ReadNMEA(%s) => %d\n", NMEA, Err);
  }
  if(IGC) { fclose(IGC); IGC=0; }
  fclose(File);
  return 0; }

int main(int argc, char *argv[])
{
  for(int arg=1; arg<argc; arg++)
  { ProcessFile(argv[arg]); }

  return 0; }

// ===========================================================================================

