#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <time.h>

#include <vector>
#include <algorithm>

#include "ogn1.h"
#include "ogn.h"

static const char *File_MAC_ID_Time(uint64_t &MAC, uint32_t &ID, uint32_t &Time, const char *FileName)
{ // if(FileName==0) printf("File_MAC_ID_Time( , , , NULL)\n");
  //            else printf("File_MAC_ID_Time( , , , \"%s\")\n", FileName);
  MAC=0; ID=0; Time=0;
  if(FileName==0) return 0;

  const char *ShortName=FileName;
  const char *Slash =strrchr(FileName , '/'); if(Slash) ShortName=Slash+1;         // search for a slash is path is given

  // printf(" Slash = %s\n", Slash);
  // printf(" ShortName = %s\n", ShortName);

  const char *Uscore=strrchr(ShortName, '_'); // if(Uscore) ShortName=Uscore+1;     // search for an underscore which separates fields

  // printf(" Uscore = %s\n", Uscore);

  if(Uscore && strlen(ShortName)==34)
  { Read_Hex(MAC , ShortName   , 12); // printf("Cannot not read Tracker MAC: %s\n", ShortName);
    Read_Hex(ID  , ShortName+13,  8); // printf("Cannot not read Aircraft-ID: %s\n", ShortName);
    Read_Hex(Time, ShortName+22,  8);
  }
  else if(strlen(ShortName)==12)
    Read_Hex(Time, ShortName, 8); // printf("Not a TLG file: %s\n", ShortName);

  // printf(" MAC:ID:Time: %.12s %.8s %.8s => %012lX %08X %08X\n",
  //    ShortName, ShortName+13, ShortName+22, MAC, ID, Time);

  return ShortName; }

// static bool FileLess(const char *File1, const char *File2)
// { return strcmp(File1, File2)<0; }

static bool FileLess(const char *File1, const char *File2)
{ // printf("%s <=> %s\n", File1, File2);
  uint64_t MAC1; uint32_t ID1, Time1; File_MAC_ID_Time(MAC1, ID1, Time1, File1);
  uint64_t MAC2; uint32_t ID2, Time2; File_MAC_ID_Time(MAC2, ID2, Time2, File2);
       if(MAC1<MAC2) return 1;
  else if(MAC1>MAC2) return 0;
       if(ID1<ID2) return 1;
  else if(ID1>ID2) return 0;
  return Time1<Time2; }

static char Line[512];

static int ProcessFile(const char *FileName)
{
  uint64_t MAC=0;
  uint32_t AcftID=0;
  uint32_t FileTime=0;
  const char *ShortName=File_MAC_ID_Time(MAC, AcftID, FileTime, FileName); if(ShortName==0) return 0;
  time_t Time = FileTime;
  struct tm *TM = gmtime(&Time);
  printf("MAC:%012lX ID:%08X Time:%04d.%02d.%02d %02d:%02d:%02d File:%s\n",
           MAC, AcftID, TM->tm_year+1900, TM->tm_mon+1, TM->tm_mday, TM->tm_hour, TM->tm_min, TM->tm_sec, ShortName);

  FILE *File = fopen(FileName, "rb"); if(File==0) { printf("Cannot open %s for read\n", FileName); return 0; }
  OGN_LogPacket<OGN1_Packet> Packet;
  uint32_t UpdTime=FileTime;
  int Packets=0;
  for( ; ; )
  { if(fread(&Packet, Packet.Bytes, 1, File)!=1) break;     // read the next packet from the file
    if(!Packet.isCorrect()) continue;                       //
    uint32_t Time=Packet.getTime(UpdTime);                  // [sec] get full time from short time in the aprox. full time
    int Len=Packet.Packet.WriteAPRS(Line, Time);
    if(Len==0) continue;
    UpdTime=Time;
    printf("%s\n", Line); }
  fclose(File); return Packets; }

static std::vector<const char *> FileList;

int main(int argc, char *argv[])
{

  for(int arg=1; arg<argc; arg++)
  { FileList.push_back(argv[arg]); }

  // printf("%lu files\n", FileList.size());
  std::sort(FileList.begin(), FileList.end(), FileLess);
  // printf("Sort done\n");

  for(size_t Idx=0; Idx<FileList.size(); Idx++)
  { ProcessFile(FileList[Idx]); }

  return 0; }

