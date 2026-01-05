#include <stdio.h>

#include "rx-pkt.h"

// ===================================================================================================

#include "format.h"

static int ReadHex(uint8_t *Data, int MaxBytes, const char *Inp) // read from an ASCII string
{ uint8_t Len=0;
  for( ; Len<MaxBytes; )
  { int8_t Upp=Read_Hex1(*Inp++); if(Upp<0) break;
    int8_t Low=Read_Hex1(*Inp++); if(Low<0) break;
    Data[Len++] = (Upp<<4) | Low; }
return Len; }

static void PrintHex(const uint8_t *Data, int Len)
{ for(int Idx=0; Idx<Len; Idx++)
    printf("%02X", Data[Idx]); }

// ===================================================================================================

static uint8_t Data[64];

const char *Input = "9B2B640564385DF2467BF6A1936A7B3CA1E5A9820245B4150E08A10E20";
const char *Output = "80AC870BBE48CF7ED4326D4F67943CB5304048B682A1C1140E08";

OGN_TxPacket<OGN1_Packet> Packet;

int main(int argc, char *argv[])
{

  int InpLen=ReadHex(Data, 64, Input);
  printf("Input [%d] ", InpLen);
  PrintHex(Data, InpLen);
  printf("\n");

  int OutLen=FSK_RxPacket::BitShift(Data, InpLen, 21);

  printf("Output[%d] ", OutLen);
  PrintHex(Data, OutLen);
  printf("\n");

  memcpy(&Packet, Data, Packet.Bytes);
  int CheckErr=Packet.checkFEC();
  printf("checkFEC() => %d\n", CheckErr);
  Packet.calcFEC();
  Packet.Dump();

  return 0; }
