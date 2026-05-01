#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <unistd.h>
#include <time.h>
#include <string.h>
#include <math.h>

#include <vector>

#include "serial.h"
#include "ogn.h"

// =====================================================================================================================

static double getTime(void)                                           // read the system time at this very moment
{ struct timespec now; clock_gettime(CLOCK_REALTIME, &now); return now.tv_sec + 1e-9*now.tv_nsec; }

// =====================================================================================================================

static const char *PortName = "/dev/ttyUSB0";                              // default serial port name
static int         BaudRate = 115200;                                      // default baud rate on the serial port
static int         ListOnly = 0;
static uint32_t    SelectFile  = 0;
static int         Help = 0;

static SerialPort Port;

static char CmdLine[64];
const int MaxLineLen = 512;
static char RxLine[MaxLineLen];

static  int64_t DevEUI     = 0;                   // 64-bit device EUI = tracker MAC
static  uint32_t Address = 0;
static  uint8_t AddrType = 0;
static  uint8_t AcftType = 0;

static std::vector<uint32_t> LogFileList;         // list of log file start times

// =====================================================================================================================
// const int MaxLineLen=256;
static char InpLine[MaxLineLen];
static int InpLineLen=0;
static bool GoodLine=1;

static char *getInpLine(void)
{ char Byte=0;
  for( ; ; )
  { if(InpLineLen>=MaxLineLen) { InpLineLen=0; GoodLine=0; }
    int Read=Port.Read(&Byte, 1); if(Read<=0) return 0;
    if(Byte<' ') break;
    InpLine[InpLineLen++]=Byte; }
  if(!GoodLine) { GoodLine=1; InpLineLen=0; return 0; }
  InpLine[InpLineLen]=0;
  if(InpLineLen==0) return 0;
  InpLineLen=0; return InpLine; }

static char *WaitResp(const char *Match, double Timeout=1.0)
{ double StartTime=getTime();
  for( ; ; )
  { if(getTime()-StartTime>=Timeout) break;
    char *RxLine=getInpLine();
    if(RxLine==0) { usleep(1000); continue; }
    if(strstr(RxLine, Match)) return RxLine; }
  return 0; }

// =====================================================================================================================

static int List(void)
{
  sleep(1);
  Port.Write(12);                                                     // send Ctrl-L to list the log files

  uint8_t Index[32];
  int LineIdx=0;
  double Start=getTime();                                             // sart counting the time
  for( ; ; )
  { char Byte;
    if(Port.Read(Byte)<=0)                                            // get a byte from the serial port
    { double Now=getTime();                                           // if non, then check time
      if((Now-Start)>=5.0) break;                                     // if idle for more than 4 sec then stop recording the log files
      usleep(1000); continue; }                                       // if no new bytes on the serial port sleep a little
    // printf("%3d: %02X %c\n", LineIdx, Byte, Byte<=' '?' ':Byte);
    if(Byte<' ')                                                      // if a control (non-printable) character
    { RxLine[LineIdx]=0;                                                // terminate the line and process it
      if(LineIdx<7) { LineIdx=0; continue; }
      // printf("%s\n", RxLine);
      if(memcmp(RxLine, "$POGNL,", 7)!=0) { LineIdx=0; continue; }      // only take POGNL sentences
      int8_t Err=GPS_Position::IndexNMEA(Index, RxLine);              // index the parameters
      if(Err<2) { LineIdx=0; continue; }                              // if less than two parameters, then skip this message
      if((Index[1]-Index[0])!=13) { LineIdx=0; continue; }            // filename must be 13 characters long
      uint32_t UnixTime=0;
      if(Read_Hex(UnixTime, RxLine+Index[0])!=8) { LineIdx=0; continue; }   // get the starting time: 8 hex characters
      time_t Time = UnixTime;
      struct tm *TM = gmtime(&Time);
      printf("%s (%d,%d) %02d:%02d:%02d %02d.%02d.%04d\n",
             RxLine, Err, Index[1]-Index[0], TM->tm_hour, TM->tm_min, TM->tm_sec, TM->tm_mday, TM->tm_mon+1, TM->tm_year+1900);
      Start=getTime();                                                // set new start time counter
      LogFileList.push_back(Time);
      LineIdx=0; continue; }
    if(LineIdx<MaxLineLen) RxLine[LineIdx++]=Byte;                      // add the byte to the line, keep collecting more bytes
  }
  printf("%u log files\n", (unsigned)LogFileList.size());

  return LogFileList.size(); }

// =====================================================================================================================

static int Download(uint32_t LogFile, bool Save=1)
{ char Cmd[128];

  FILE *SaveFile = 0;
  if(Save)
  { sprintf(Cmd, "%012lX_%08X.TLG.aprs", DevEUI, LogFile);
    SaveFile = fopen(Cmd, "wt");
    if(SaveFile) fprintf(SaveFile, "# %08X.TLG 0x%012lX %X:%d:%0x06X\n",
                     LogFile, DevEUI, AcftType, AddrType, Address);
  }

  sleep(1);
  sprintf(Cmd, "$POGNL,%08X.TLG\n", LogFile);
  Port.Write(Cmd);
  printf("Port <- %s", Cmd);

  int Records=0;
  int LineIdx=0;
  double Start=getTime();
  for( ; ; )
  { char Byte;
    if(Port.Read(Byte)<=0)
    { double Now=getTime();
      if((Now-Start)>=4.0) break;
      usleep(1000); continue; }                                       // if no new bytes on the serial port sleep a little
    if(Byte<' ')                                                      // if a control (non-printable) character
    { RxLine[LineIdx]=0;
      if(RxLine[0]=='$') { LineIdx=0; continue; }
      if(LineIdx<12) { LineIdx=0; continue; }
      printf("%s\n", RxLine);
      if(SaveFile) fprintf(SaveFile, "%s\n", RxLine);
      Start=getTime();
      Records++;
      LineIdx=0; continue; }
    if(LineIdx<MaxLineLen) RxLine[LineIdx++]=Byte;
  }

  printf("%d log records for %08X.TLG\n", Records, LogFile);

  if(SaveFile) fclose(SaveFile);
  return Records; }

// =====================================================================================================================

int main(int argc, char *argv[])
{

  int arg=1;
  for( ; arg<argc; arg++)
  { const char *Val = argv[arg]; if(Val[0]!='-') break;
    switch(Val[1])
    { case 'h': Help=1; break;
      case 'l': ListOnly=1; break;
      case 'f': SelectFile=strtol(Val+2, 0, 16); break;
      default: Help=1; break;
    }
  }

  if(Help)
  { printf("Usage: %s [options] <serial-port:baudrate>\n\
Options: -h                this help\n\
         -l                only list the log files stored in the OGN-Tracker\n\
         -f<UTC-hex-time>  download the selected file: use the UTC time\n\
", argv[0]);
    return 0; }

  if(arg<argc)
  { char *Colon = strchr(argv[arg],':');
    if(Colon) { Colon[0]=0; BaudRate=atoi(Colon+1); }
    PortName = argv[arg]; }

  if(Port.Open(PortName, BaudRate)<0) { printf("Can't open %s at %dbps\n", PortName, BaudRate); return -1; }
  // printf("Open %s at %dbps\n", PortName, BaudRate);

  if(SelectFile==0) List();

  if(ListOnly) { Port.Close(); return 0; }

                                            // Read the MAC from the tracker
  for( int Try=0; Try<3; Try++)             // retry three times
  { strcpy(CmdLine, "$POGNS\r\n");          // prompt the tracker to send its MAC and other parameters
    Port.Write(CmdLine);
    printf("Serial <= %s", CmdLine);
    for( ; ; )
    { char *RxLine = WaitResp("$POGNS", 1.0); if(RxLine==0) break;
      char *Parm=strstr(RxLine, "CPU=");
      if(Parm) { sscanf(Parm+4, "%lX", &DevEUI); }
      Parm=strstr(RxLine, "Address=");
      if(Parm) { Address=strtol(Parm+8, 0, 0); }
      Parm=strstr(RxLine, "AddrType=");
      if(Parm) { AddrType=strtol(Parm+9, 0, 0); }
      Parm=strstr(RxLine, "AcftType=");
      if(Parm) { AcftType=strtol(Parm+9, 0, 0); }
      printf("Serial => %s\n", RxLine); }
    if(DevEUI>0xFFFFFFFF) break; }
  // if(DevEUI<=0xFFFFFFFF) return 0;

  if(SelectFile)
  { Download(SelectFile); }
  else
  { for( size_t File=0; File<LogFileList.size(); File++)
      Download(LogFileList[File]); }

  Port.Close();
  return 0; }

