#include <stdio.h>
#include <stdint.h>

#include "gps-satlist.h"

GPS_SatList SatList;

int main(int argc, char *argv[])
{
  FILE *InpFile=stdin;
  if(argc>1)
  { InpFile=fopen(argv[1], "rt");
    if(InpFile==0) { printf("Cannot open %s for read\n", argv[1]); return -1; }
  }

  NMEA_RxMsg NMEA;
  char Line[128];
  for( ; ; )
  { if(fgets(Line, 128, InpFile)==0) break;
    int Ret=NMEA.ProcessLine(Line);
    printf("(%3d:%c:%2d) %s", Ret, NMEA.isComplete()?'+':'-', NMEA.Parms, Line);
    if(NMEA.isComplete())
    { SatList.Process(NMEA);
      if(NMEA.isGxRMC())
      { SatList.Sort();
        SatList.PrintSats();
        SatList.CalcStats();
        SatList.PrintStats(Line);
        printf("%s\n", Line); }
    }
  }

  if(InpFile!=stdin) fclose(InpFile);
  return 0; }

