#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <math.h>
#include <sys/random.h>

#include "serial.h"
#include "nmea.h"

// ==================================================================================================================

static double getTime(clockid_t ClockID=CLOCK_BOOTTIME) // read the system time at this very moment
{ struct timespec Now; clock_gettime(ClockID, &Now); return Now.tv_sec + 1e-9*Now.tv_nsec; }

// ==================================================================================================================

static int PrintHex(char *Out, const uint8_t *Byte, int Len, const char *Format="%02X")
{ int OutLen=0;
  for(int Idx=0; Idx<Len; Idx++)
    OutLen+=sprintf(Out+OutLen, Format, Byte[Idx]);
  return OutLen; }

// ==================================================================================================================

static SerialPort Port;

// write given number of Data bytes to the serial port
static int SerialWrite(const uint8_t *Data, int Len)
{ int Ofs=0;
  for( ; Ofs<Len; )
  { int Write=Port.Write((const char *)(Data+Ofs), Len-Ofs);
    if(Write<0) return Write;
    if(Write==0) { usleep(1000); continue; }
    Ofs+=Write; }
  return Ofs; }

// read Data bytes from the serial port, with Timeout [sec]
static int SerialRead(uint8_t *Data, int Len, double Timeout)
{ int Ofs=0;
  double StartTime=getTime();
  for( ; Ofs<Len; )
  { int Read=Port.Read((char *)(Data+Ofs), Len-Ofs);
    if(Read<0) return Read;
    if(Read==0)
    { usleep(1000); if(getTime()-StartTime>=Timeout) break;
      continue; }
    Ofs+=Read; }
  return Ofs; }  // return the number of bytes actually read

const int MaxLineLen=256;
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

// ==================================================================================================================

static char CmdLine[512];

int main(int argc, char *argv[])
{
  const char *PortName = "/dev/ttyACM0";                              // default serial port name
        int   BaudRate = 115200;                                      // default baud rate on the serial port
  if(argc>1)
  { char *Colon = strchr(argv[1],':');
    if(Colon) { Colon[0]=0; BaudRate=atol(Colon+1); }
    PortName = argv[1]; }
  if(Port.Open(PortName, BaudRate)<0) { printf("Can't open %s at %dbps\n", PortName, BaudRate); return -1; }

  int64_t JoinEUI    = 0x70B3D57ED0035895;  // OGN application EUI
  int64_t DevEUI     = 0;
  char    DevID[40];
  uint8_t AppKey[16];

  for( int Try=0; Try<3; Try++)
  { strcpy(CmdLine, "$POGNS\r\n");
    Port.Write(CmdLine);
    printf("Serial <= %s", CmdLine);
    for( ; ; )
    { char *RxLine = WaitResp("$POGNS", 1.0); if(RxLine==0) break;
      char *CPU=strstr(RxLine, "CPU=");
      if(CPU) { sscanf(CPU+4, "%lX", &DevEUI); }
      printf("Serial => %s\n", RxLine); }
    if(DevEUI>0xFFFFFFFF) break; }
  if(DevEUI<=0xFFFFFFFF) return 0;

  if(getrandom(&AppKey, 16, 0)!=16) { printf("Could not produce AppKey\n"); return 0; }
  sprintf(DevID, "ogntrk-%012lx", DevEUI);
  PrintHex(CmdLine, AppKey, 16);
  printf("JoinEUI=%016lX, DevEUI=%012lX, DevID=%s, AppKey=%s\n", JoinEUI, DevEUI, DevID, CmdLine);

  int CmdLen=sprintf(CmdLine, "ttn-lw-stack.ttn-lw-cli devices delete --application-id=ogn --device-id=%s\n", DevID);
  int CmdRet=system(CmdLine);
  printf("%s => %d\n", CmdLine, CmdRet);

  CmdLen=sprintf(CmdLine, "ttn-lw-stack.ttn-lw-cli devices create --application-id=ogn --device-id=%s", DevID);
  CmdLen+=sprintf(CmdLine+CmdLen, " --dev-eui %016lX --join-eui %016lX --root-keys.app-key.key ", DevEUI, JoinEUI);
  CmdLen+=PrintHex(CmdLine+CmdLen, AppKey, 16);
  CmdLen+=sprintf(CmdLine+CmdLen, " --frequency-plan-id EU_863_870 --lorawan-version 1.0.3 --lorawan-phy-version 1.0.3-a");
  // CmdLen+=sprintf(CmdLine+CmdLen, " --mac-settings.supports-32-bit-f-cnt true");
  CmdLen+=sprintf(CmdLine+CmdLen, " --mac-settings.status-time-periodicity 0 --mac-settings.status-count-periodicity 0");
  CmdLen+=sprintf(CmdLine+CmdLen, " --mac-settings.adr.mode.disabled");
  CmdLen+=sprintf(CmdLine+CmdLen, "\n");

  CmdRet=system(CmdLine);
  printf("%s => %d\n", CmdLine, CmdRet);

  CmdLen=sprintf(CmdLine, "$POGNS,AppKey=");
  CmdLen+=PrintHex(CmdLine+CmdLen, AppKey, 16);
  CmdLen+=sprintf(CmdLine+CmdLen, "\r\n");
  Port.Write(CmdLine);
  printf("Serial <= %s", CmdLine);

  Port.Close();
  return 0; }

