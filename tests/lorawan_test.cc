#include <stdio.h>

#include "lorawan.h"

static LoRaWANnode TTN;

int main(int argc, char *argv[])
{
  printf("sizeof(LoRaWANnode)=%u\n", (unsigned)sizeof(LoRaWANnode));
  return 0; }

