#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <strings.h>
#include <ctype.h>
#include <math.h>

#include <string>
#include <vector>
#include <algorithm>

struct TrackPoint
{ double Lat, Lon, Alt;
  bool HasAlt, HasTime;
  bool HasSat, HasDOP;
  int Satellites;
  double PDOP, HDOP, VDOP;
  int Year, Month, Day, Hour, Minute, Second;
  unsigned long Order;
};

struct SatReport
{ int Hour, Minute, Second, Satellites;
  double PDOP, HDOP, VDOP;
  unsigned long Order;
};

static int HexValue(char c)
{ if(c>='0' && c<='9') return c-'0';
  if(c>='A' && c<='F') return c-'A'+10;
  if(c>='a' && c<='f') return c-'a'+10;
  return -1; }

static bool IsHexString(const char *Text)
{ if(!*Text) return false;
  for( ; *Text; Text++) if(HexValue(*Text)<0) return false;
  return true; }

static bool ParseCoordinate(const char *Text, int DegreesDigits, char Positive, char Negative,
                            double &Value, int &Consumed)
{
  // APRS coordinates are DDMM.mmH or DDDMM.mmH.
  for(int Idx=0; Idx<DegreesDigits+2; Idx++) if(!isdigit((unsigned char)Text[Idx])) return false;
  if(Text[DegreesDigits+2]!='.') return false;
  if(!isdigit((unsigned char)Text[DegreesDigits+3]) || !isdigit((unsigned char)Text[DegreesDigits+4])) return false;
  char Hemisphere=Text[DegreesDigits+5];
  if(Hemisphere!=Positive && Hemisphere!=Negative) return false;
  int Degrees=0, MinutesWhole=0, MinutesFrac=0;
  for(int Idx=0; Idx<DegreesDigits; Idx++) Degrees=Degrees*10+(Text[Idx]-'0');
  MinutesWhole=(Text[DegreesDigits]-'0')*10+(Text[DegreesDigits+1]-'0');
  MinutesFrac=(Text[DegreesDigits+3]-'0')*10+(Text[DegreesDigits+4]-'0');
  double Minutes=MinutesWhole+MinutesFrac/100.0;
  // The !Wxy! extension adds one 0.001-minute digit: x for latitude, y for longitude.
  Value=Degrees+Minutes/60.0;
  if(Hemisphere==Negative) Value=-Value;
  Consumed=DegreesDigits+6;
  return Minutes<60 && Degrees<=(DegreesDigits==2 ? 90 : 180);
}

static bool ParseTime(const char *Line, int &Hour, int &Minute, int &Second)
{ const char *Body=strchr(Line, ':'); if(!Body) return false;
  const char *Stamp=Body+1;
  if(*Stamp!='/' && *Stamp!='>') return false;
  Stamp++;
  for(int Idx=0; Idx<6; Idx++) if(!isdigit((unsigned char)Stamp[Idx])) return false;
  if(Stamp[6]!='h') return false;
  Hour=(Stamp[0]-'0')*10+Stamp[1]-'0';
  Minute=(Stamp[2]-'0')*10+Stamp[3]-'0';
  Second=(Stamp[4]-'0')*10+Stamp[5]-'0';
  return Hour<24 && Minute<60 && Second<60;
}

static bool MatchesTarget(const char *Line, const char *Target)
{ const char *End=strchr(Line, '>'); if(!End) return false;
  size_t SourceLen=(size_t)(End-Line), TargetLen=strlen(Target);
  if(SourceLen==TargetLen && strncasecmp(Line, Target, TargetLen)==0) return true;
  if((TargetLen==6 || TargetLen==8) && IsHexString(Target))
  { const char *Id=strstr(End, " id");
    if(Id && strlen(Id+3)>=8)
    { const char *Address=Id+3;
      for(int Idx=0; Idx<8; Idx++) if(HexValue(Address[Idx])<0) return false;
      if(strncasecmp(Address+8-TargetLen, Target, TargetLen)==0) return true; }
    const char *ShortAddress=Target+(TargetLen==8 ? 2 : 0);
    if(SourceLen>=6 && strncasecmp(End-6,ShortAddress,6)==0) return true;
  }
  return false;
}

static bool ParseSatReport(const char *Line, SatReport &Report)
{ const char *SatSNR=strstr(Line,"SatSNR:");
  const char *DOP=SatSNR ? strstr(SatSNR,"DOP:") : 0;
  if(!SatSNR || !DOP) return false;
  if(!ParseTime(Line,Report.Hour,Report.Minute,Report.Second)) return false;
  static const char *System[]={"QZ:","GP:","GL:","GA:","BD:"};
  Report.Satellites=0;
  for(size_t Idx=0; Idx<sizeof(System)/sizeof(System[0]); Idx++)
  { const char *Entry=strstr(SatSNR,System[Idx]);
    if(Entry && Entry<DOP)
    { char *End=0; long FixSats=strtol(Entry+3,&End,10);
      if(End!=Entry+3 && *End==':' && FixSats>=0) Report.Satellites+=(int)FixSats;
    }
  }
  return sscanf(DOP+4,"%lf/%lf/%lf",&Report.PDOP,&Report.HDOP,&Report.VDOP)==3;
}

static bool ParsePoint(const char *Line, TrackPoint &Point)
{ const char *Body=strchr(Line, ':'); if(!Body) return false;
  const char *Stamp=strchr(Body, '/'); if(!Stamp || strlen(Stamp)<27) return false;
  Stamp++;
  if(Stamp[6]!='h') return false;
  const char *Pos=Stamp+7;
  int LatLen=0, LonLen=0;
  if(!ParseCoordinate(Pos, 2, 'N', 'S', Point.Lat, LatLen)) return false;
  Pos+=LatLen;
  if(!*Pos) return false; // APRS symbol between latitude and longitude.
  Pos++;
  if(!ParseCoordinate(Pos, 3, 'E', 'W', Point.Lon, LonLen)) return false;
  const char *Extra=strstr(Pos+LonLen," !W");
  if(Extra && isdigit((unsigned char)Extra[3]) &&
     isdigit((unsigned char)Extra[4]) && Extra[5]=='!')
  { double LatFraction=(Extra[3]-'0')/60000.0;
    double LonFraction=(Extra[4]-'0')/60000.0;
    Point.Lat += Point.Lat<0 ? -LatFraction : LatFraction;
    Point.Lon += Point.Lon<0 ? -LonFraction : LonFraction;
  }
  Point.HasAlt=false; Point.Alt=0;
  const char *Altitude=strstr(Pos+LonLen, "/A=");
  if(Altitude)
  { int32_t Feet=0; int Count=0;
    for(const char *p=Altitude+3; Count<6 && isdigit((unsigned char)*p); p++,Count++) Feet=Feet*10+(*p-'0');
    if(Count==6) { Point.Alt=Feet*0.3048+40.0; Point.HasAlt=true; }
  }
  Point.HasTime=ParseTime(Line, Point.Hour, Point.Minute, Point.Second);
  return true;
}

static bool ValidDate(int Year, int Month, int Day)
{ if(Month<1 || Month>12 || Day<1) return false;
  static const int Days[]={31,28,31,30,31,30,31,31,30,31,30,31};
  int Max=Days[Month-1];
  if(Month==2 && (Year%4==0 && (Year%100!=0 || Year%400==0))) Max++;
  return Day<=Max;
}

static void NextDay(int &Year, int &Month, int &Day)
{ Day++;
  while(!ValidDate(Year,Month,Day))
  { Day=1; Month++;
    if(Month>12) { Month=1; Year++; }
  }
}

static std::string XmlEscape(const char *Text)
{ std::string Out;
  for( ; *Text; Text++)
  { switch(*Text)
    { case '&': Out+="&amp;"; break;
      case '<': Out+="&lt;"; break;
      case '>': Out+="&gt;"; break;
      case '\"': Out+="&quot;"; break;
      case '\'': Out+="&apos;"; break;
      default: Out+=*Text; }
  }
  return Out;
}

int main(int argc, char *argv[])
{ if(argc!=3)
  { fprintf(stderr, "Usage: %s <APRS-name-or-address> <input-file.aprs|->\n", argv[0]);
    return 2; }
  const char *Target=argv[1];
  const char *InputName=argv[2];
  std::string OutputName=std::string(Target)+".gpx";
  FILE *Input=strcmp(InputName,"-")==0 ? stdin : fopen(InputName,"rt");
  if(!Input) { fprintf(stderr,"Cannot open %s for read\n",InputName); return 1; }

  std::vector<TrackPoint> Points;
  std::vector<SatReport> SatReports;
  char Line[2048];
  int InputLines=0, MatchedLines=0;
  unsigned long Order=0;
  while(fgets(Line,sizeof(Line),Input))
  { InputLines++;
    char *EOL=strchr(Line,'\n'); if(EOL) *EOL=0;
    size_t Len=strlen(Line); if(Len && Line[Len-1]=='\r') Line[Len-1]=0;
    if(!MatchesTarget(Line,Target)) continue;
    MatchedLines++;
    SatReport Sat;
    if(ParseSatReport(Line,Sat))
    { Sat.Order=Order++; SatReports.push_back(Sat); continue; }
    TrackPoint Point;
    if(!ParsePoint(Line,Point)) continue;
    Point.HasSat=false; Point.HasDOP=false;
    Point.Order=Order++;
    Points.push_back(Point);
  }
  if(Input!=stdin) fclose(Input);
  if(Points.empty())
  { fprintf(stderr,"No position records found for %s in %s (%d matching APRS lines)\n",Target,InputName,MatchedLines);
    return 1; }

  // APRS logs can contain receiver batches appended out of timestamp order. Sort by UTC time,
  // then start after the largest daily gap so a night crossing midnight stays on one date span.
  std::stable_sort(Points.begin(),Points.end(),[](const TrackPoint &A, const TrackPoint &B)
  { if(A.HasTime!=B.HasTime) return A.HasTime;
    if(!A.HasTime) return A.Order<B.Order;
    int TimeA=A.Hour*3600+A.Minute*60+A.Second;
    int TimeB=B.Hour*3600+B.Minute*60+B.Second;
    if(TimeA!=TimeB) return TimeA<TimeB;
    return A.Order<B.Order;
  });
  size_t Start=0;
  int LargestGap=-1;
  for(size_t Idx=0; Idx<Points.size(); Idx++)
  { if(!Points[Idx].HasTime) break;
    size_t Next=(Idx+1)%Points.size();
    if(!Points[Next].HasTime) break;
    int This=Points[Idx].Hour*3600+Points[Idx].Minute*60+Points[Idx].Second;
    int After=Points[Next].Hour*3600+Points[Next].Minute*60+Points[Next].Second;
    int Gap=(After-This+86400)%86400;
    if(Gap>LargestGap) { LargestGap=Gap; Start=Next; }
  }
  std::rotate(Points.begin(),Points.begin()+Start,Points.end());
  int StartSeconds=Points[0].HasTime ? Points[0].Hour*3600+Points[0].Minute*60+Points[0].Second : 0;
  for(size_t Idx=0; Idx<Points.size(); Idx++) if(Points[Idx].HasTime)
  { TrackPoint &Point=Points[Idx];
    int PointSeconds=Point.Hour*3600+Point.Minute*60+Point.Second;
    int PointElapsed=(PointSeconds-StartSeconds+86400)%86400;
    int BestAge=600;
    for(size_t SatIdx=0; SatIdx<SatReports.size(); SatIdx++)
    { const SatReport &Sat=SatReports[SatIdx];
      int SatSeconds=Sat.Hour*3600+Sat.Minute*60+Sat.Second;
      int SatElapsed=(SatSeconds-StartSeconds+86400)%86400;
      int Age=(PointElapsed-SatElapsed+86400)%86400;
      if(Age<BestAge)
      { BestAge=Age;
        Point.Satellites=Sat.Satellites; Point.HasSat=true;
        Point.PDOP=Sat.PDOP; Point.HDOP=Sat.HDOP; Point.VDOP=Sat.VDOP; Point.HasDOP=true;
      }
    }
  }
  int Year=2026, Month=11, Day=16, LastSeconds=-1;
  for(size_t Idx=0; Idx<Points.size(); Idx++) if(Points[Idx].HasTime)
  { TrackPoint &Point=Points[Idx];
    int Seconds=Point.Hour*3600+Point.Minute*60+Point.Second;
    if(LastSeconds>=0 && Seconds<LastSeconds) NextDay(Year,Month,Day);
    LastSeconds=Seconds;
    Point.Year=Year; Point.Month=Month; Point.Day=Day;
  }

  FILE *Output=fopen(OutputName.c_str(),"wt");
  if(!Output) { fprintf(stderr,"Cannot open %s for write\n",OutputName.c_str()); return 1; }
  std::string SafeTarget=XmlEscape(Target);
  fprintf(Output,"<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n");
  fprintf(Output,"<gpx version=\"1.1\" creator=\"aprs2gpx\" xmlns=\"http://www.topografix.com/GPX/1/1\">\n");
  fprintf(Output,"  <metadata><name>%s</name></metadata>\n",SafeTarget.c_str());
  fprintf(Output,"  <trk><name>%s</name><trkseg>\n",SafeTarget.c_str());
  for(size_t Idx=0; Idx<Points.size(); Idx++)
  { const TrackPoint &Point=Points[Idx];
    fprintf(Output,"    <trkpt lat=\"%.7f\" lon=\"%.7f\">\n",Point.Lat,Point.Lon);
    if(Point.HasAlt) fprintf(Output,"      <ele>%.1f</ele>\n",Point.Alt);
    if(Point.HasTime) fprintf(Output,"      <time>%04d-%02d-%02dT%02d:%02d:%02dZ</time>\n",
                              Point.Year,Point.Month,Point.Day,Point.Hour,Point.Minute,Point.Second);
    if(Point.HasSat) fprintf(Output,"      <sat>%d</sat>\n",Point.Satellites);
    if(Point.HasDOP)
    { fprintf(Output,"      <hdop>%.1f</hdop>\n",Point.HDOP);
      fprintf(Output,"      <vdop>%.1f</vdop>\n",Point.VDOP);
      fprintf(Output,"      <pdop>%.1f</pdop>\n",Point.PDOP);
    }
    fprintf(Output,"    </trkpt>\n");
  }
  fprintf(Output,"  </trkseg></trk>\n</gpx>\n");
  fclose(Output);
  size_t PointsWithSat=0;
  for(size_t Idx=0; Idx<Points.size(); Idx++) if(Points[Idx].HasSat) PointsWithSat++;
  printf("%d lines from %s; %d matching APRS lines; %lu track points (%lu with recent SatSNR) to %s\n",
         InputLines,InputName,MatchedLines,(unsigned long)Points.size(),(unsigned long)PointsWithSat,OutputName.c_str());
  return 0;
}
