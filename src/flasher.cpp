#include "main.h"

#include "flasher.h"
#include "gps.h"
#include <fifo.h>

#ifdef WITH_FLASHER

void Flasher_Init(void)
{
#ifdef Flasher_Pin
  pinMode(Flasher_Pin, OUTPUT);
  digitalWrite(Flasher_Pin, LOW);
#endif
}

void Flasher_ON(bool ON)
{
#ifdef Flasher_Pin
  digitalWrite(Flasher_Pin, ON);
#endif
}

const  uint8_t Flasher_BitPeriod       = 10;   // [ms]
const uint32_t Flasher_IdleFlashPeriod = 30; // [sec]

static uint32_t Flasher_LastActive = 0;
static uint32_t Flasher_Pattern   =  0;
static uint8_t Flasher_PattBits   =  0;
static uint8_t Flasher_Counter    =  0;

static FIFO<uint32_t,  8> Flasher_FIFO;

void Flasher_Play(uint32_t Patt) { Flasher_FIFO.Write(Patt); }

void Flasher_TimerCheck(uint8_t Ticks)
{ uint32_t msTime=millis();
  if(Ticks<Flasher_Counter) { Flasher_Counter-=Ticks; return; }
  Ticks-=Flasher_Counter;
  bool ON=0;
  if(Flasher_PattBits==0)
  { if(Flasher_FIFO.Read(Flasher_Pattern)>0)
    { Flasher_PattBits=32; Flasher_LastActive=msTime; }
    else
    { uint32_t Idle=msTime-Flasher_LastActive;
      if(Flight.inFlight() && Idle>=Flasher_IdleFlashPeriod*1000)
      { Flasher_Pattern=Flasher_PattDouble; Flasher_PattBits=32; Flasher_LastActive=msTime; }
    }
  }
  if(Flasher_PattBits>0) { ON = Flasher_Pattern&0x80000000; Flasher_Pattern<<=1; Flasher_PattBits--; }
  Flasher_ON(ON);
  Flasher_Counter=Flasher_BitPeriod+Ticks; }

#endif // WITH_FLASHER
