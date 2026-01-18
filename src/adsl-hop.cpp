#include "adsl-hop.h"

// =====================================================================================================================
// here is all the coding needed for the aircraft to decide on which channel to transmit/receive

static uint8_t BitRev(uint8_t Byte)                   // reverse bits in a 6-bit byte
{ Byte = ((Byte&0b00111000)>>3) | ((Byte&0b00000111)<<3);
  Byte = ((Byte&0b00100100)>>2) | ((Byte&0b00001001)<<2) | (Byte&0b00010010);
  return Byte; }

static uint8_t Scramble(uint8_t Sec)                  // [0..59] scramble a second
{ Sec=BitRev(Sec);
  if(Sec<60) return Sec;
  return Sec-60; }

uint8_t ADSL_HopChannel(uint8_t Sec, int32_t Alt)     // decide on the channel to hop based on Second and Altitude
{ if(Alt<0) Alt=0;                                    // protect against negative altitides
  if(Sec>=60) Sec-=60;                                // protect against second out of range (like leap second)
  uint16_t AltBand = Alt/100;                         // [100m] altitude band to select hopping pattern
  uint8_t HopPhase = AltBand%60;                      // [0..59] slot phase depends on the altitude band
  uint8_t ScrSec = Scramble(Sec);                     // [0..59] scrambled second
  uint8_t ChSec = ScrSec+HopPhase; if(ChSec>=60) ChSec-=60;
  uint8_t Chan = ChSec/15;                            // [0..3]
  return Chan ^ (Chan>>1); }                          // [0..3] M-Band-only devices can simply take the lowest bit

// =====================================================================================================================
