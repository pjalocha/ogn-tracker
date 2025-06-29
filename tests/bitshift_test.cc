#include <stdio.h>

#include "rx-pkt.h"


static uint8_t Data[4] = { 0x01, 0x23, 0x45, 0x67 } ;

int main(int argc, char *argv[])
{

  for(int Idx=0; Idx<4; Idx++)
   printf("%02X", Data[Idx]);
  printf("\n");

  int Len=FSK_RxPacket::BitShift(Data, 4, 13);

  for(int Idx=0; Idx<4; Idx++)
    printf("%02X", Data[Idx]);
  printf(" [%d]\n", Len);

  return 0; }


