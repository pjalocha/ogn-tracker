#ifndef __FLARM_H__
#define __FLARM_H__

#include <stdlib.h>
#include <string.h>

#include "bitcount.h"
#include "format.h"
#include "crc1021.h"

class Flarm_Packet
{ public:

  const static uint8_t Words = 6;                                   // data size, exclude CRC
  const static uint8_t Bytes = 4*Words;                             // data size, exclude CRC
  const static uint8_t DataWords = 5;                               // exclude the header
  union
  { uint32_t Word[Words];                                           // seen as 6 words (without CRC)
    uint8_t  Byte[Bytes+2];                                         // seen as 24 bytes plus 2 CRC bytes
    struct                                                          // FLARM v6 up to 2024
    { uint32_t Header;                                              // header: Address
      union
      { uint32_t Data[DataWords];                                   // Position+Prediction - this part is encrypted
      } ;
      uint8_t CRC[2];                                               // CRC-1021
      uint8_t Dummy[2];
    } ;
  } ;

   void Copy(uint8_t *Data) { memcpy(Byte, Data, Bytes); }          // only works for little-endian
   void Clear(void) { for(uint8_t Idx=0; Idx<Words; Idx++) Word[Idx]=0; }

  public:

   void PrintBin(uint32_t Word, uint8_t Bits=32) const
   { putchar(' ');
     for(int8_t Bit=(Bits-1); Bit>=0; Bit--)
     { putchar('0'+ ((Word>>(Bits-1))&1) ); Word<<=1; }
   }

  static const uint16_t SYNC_CRC = 0x051E;            // CRC of the three SYNC bytes

  void setCRC(void)
  { uint16_t CRCw  = crc1021(SYNC_CRC, Byte, Bytes);
    Byte[Bytes] = (uint8_t)(CRCw>>8); Byte[Bytes+1] = (uint8_t)CRCw; }

  uint16_t checkCRC(void) { return checkCRC(Byte, Bytes); }

  static uint16_t checkCRC(const uint8_t *Byte, uint8_t Bytes)
  { uint16_t CRCw  = crc1021(SYNC_CRC, Byte, Bytes);
    uint16_t CRCw2 = Byte[Bytes+1] | (uint16_t)(Byte[Bytes])<<8;
    return CRCw ^ CRCw2; }

  // correct the manchester-decoded packet with dead/weak bits marked
  static int Correct(uint8_t *PktData, uint8_t *PktErr, const int MaxBadBits=6)
  { uint16_t CRC = checkCRC(PktData, Bytes); if(CRC==0) return 0;
    uint8_t ErrBit=FindCRCsyndrome(CRC);
    if(ErrBit!=0xFF) { FlipBit(PktData, ErrBit); return 1; }

    uint8_t BadBitIdx[MaxBadBits];                                    // bad bit index
    uint8_t BadBitMask[MaxBadBits];                                   // bad bit mask
    uint16_t Syndrome[MaxBadBits];                                    // bad bit mask
    uint8_t BadBits=0;                                                // count the bad bits
    for(uint8_t ByteIdx=0; ByteIdx<Bytes+2; ByteIdx++)                // loop over bytes
    { uint8_t Byte=PktErr[ByteIdx];
      uint8_t Mask=0x80;
      for(uint8_t BitIdx=0; BitIdx<8; BitIdx++)                       // loop over bits
      { if(Byte&Mask)
        { if(BadBits<MaxBadBits)
          { BadBitIdx[BadBits]=ByteIdx;                               // store the bad bit index
            BadBitMask[BadBits]=Mask;
            Syndrome[BadBits]=CRCsyndrome(ByteIdx*8+BitIdx); }
          BadBits++;
        }
        Mask>>=1;
      }
      if(BadBits>MaxBadBits) break;
    }
    if(BadBits>MaxBadBits) return -1;                                 // return failure when too many bad bits

    uint8_t Loops = 1<<BadBits; uint8_t PrevGrayIdx=0;
    for(uint8_t Idx=1; Idx<Loops; Idx++)                              // loop through all combination of bad bit flips
    { uint8_t GrayIdx= Idx ^ (Idx>>1);                                // use Gray code to change flip just one bit at a time
      uint8_t BitExp = GrayIdx^PrevGrayIdx;
      uint8_t Bit=0; while(BitExp>>=1) Bit++;
      PktData[BadBitIdx[Bit]]^=BadBitMask[Bit];
      CRC^=Syndrome[Bit]; if(CRC==0) return Count1s(GrayIdx);
      uint8_t ErrBit=FindCRCsyndrome(CRC);
      if(ErrBit!=0xFF)
      { FlipBit(PktData, ErrBit);
        return Count1s(GrayIdx)+1; }
      PrevGrayIdx=GrayIdx; }

    return -1; }

  static void FlipBit(uint8_t *Byte, int BitIdx)
  { int ByteIdx=BitIdx>>3;
    BitIdx&=7; BitIdx=7-BitIdx;
    uint8_t Mask=1; Mask<<=BitIdx;
    Byte[ByteIdx]^=Mask; }

  static uint16_t CRCsyndrome(uint8_t Bit)
  { const uint16_t PacketBits = (Bytes+2)*8;
    const uint16_t Syndrome[PacketBits] = {
 0x22DA, 0x116D, 0x80A6, 0x4053, 0xA839, 0xDC0C, 0x6E06, 0x3703,
 0x9391, 0xC1D8, 0x60EC, 0x3076, 0x183B, 0x840D, 0xCA16, 0x650B,
 0xBA95, 0xD55A, 0x6AAD, 0xBD46, 0x5EA3, 0xA741, 0xDBB0, 0x6DD8,
 0x36EC, 0x1B76, 0x0DBB, 0x8ECD, 0xCF76, 0x67BB, 0xBBCD, 0xD5F6,
 0x6AFB, 0xBD6D, 0xD6A6, 0x6B53, 0xBDB9, 0xD6CC, 0x6B66, 0x35B3,
 0x92C9, 0xC174, 0x60BA, 0x305D, 0x903E, 0x481F, 0xAC1F, 0xDE1F,
 0xE71F, 0xFB9F, 0xF5DF, 0xF2FF, 0xF16F, 0xF0A7, 0xF043, 0xF031,
 0xF008, 0x7804, 0x3C02, 0x1E01, 0x8710, 0x4388, 0x21C4, 0x10E2,
 0x0871, 0x8C28, 0x4614, 0x230A, 0x1185, 0x80D2, 0x4069, 0xA824,
 0x5412, 0x2A09, 0x9D14, 0x4E8A, 0x2745, 0x9BB2, 0x4DD9, 0xAEFC,
 0x577E, 0x2BBF, 0x9DCF, 0xC6F7, 0xEB6B, 0xFDA5, 0xF6C2, 0x7B61,
 0xB5A0, 0x5AD0, 0x2D68, 0x16B4, 0x0B5A, 0x05AD, 0x8AC6, 0x4563,
 0xAAA1, 0xDD40, 0x6EA0, 0x3750, 0x1BA8, 0x0DD4, 0x06EA, 0x0375,
 0x89AA, 0x44D5, 0xAA7A, 0x553D, 0xA28E, 0x5147, 0xA0B3, 0xD849,
 0xE434, 0x721A, 0x390D, 0x9496, 0x4A4B, 0xAD35, 0xDE8A, 0x6F45,
 0xBFB2, 0x5FD9, 0xA7FC, 0x53FE, 0x29FF, 0x9CEF, 0xC667, 0xEB23,
 0xFD81, 0xF6D0, 0x7B68, 0x3DB4, 0x1EDA, 0x0F6D, 0x8FA6, 0x47D3,
 0xABF9, 0xDDEC, 0x6EF6, 0x377B, 0x93AD, 0xC1C6, 0x60E3, 0xB861,
 0xD420, 0x6A10, 0x3508, 0x1A84, 0x0D42, 0x06A1, 0x8B40, 0x45A0,
 0x22D0, 0x1168, 0x08B4, 0x045A, 0x022D, 0x8906, 0x4483, 0xAA51,
 0xDD38, 0x6E9C, 0x374E, 0x1BA7, 0x85C3, 0xCAF1, 0xED68, 0x76B4,
 0x3B5A, 0x1DAD, 0x86C6, 0x4363, 0xA9A1, 0xDCC0, 0x6E60, 0x3730,
 0x1B98, 0x0DCC, 0x06E6, 0x0373, 0x89A9, 0xCCC4, 0x6662, 0x3331,
 0x9188, 0x48C4, 0x2462, 0x1231, 0x8108, 0x4084, 0x2042, 0x1021,
 0x8000, 0x4000, 0x2000, 0x1000, 0x0800, 0x0400, 0x0200, 0x0100,
 0x0080, 0x0040, 0x0020, 0x0010, 0x0008, 0x0004, 0x0002, 0x0001 } ;

  return Syndrome[Bit]; }

  static uint8_t FindCRCsyndrome(uint16_t Syndr)              // quick search for a single-bit CRC syndrome
  { const uint16_t PacketBits = (Bytes+2)*8;
    const uint32_t Syndrome[PacketBits] = {
 0x0001CF, 0x0002CE, 0x0004CD, 0x0008CC, 0x0010CB, 0x0020CA, 0x0040C9, 0x0080C8,
 0x0100C7, 0x0200C6, 0x022D9C, 0x0373B3, 0x037567, 0x0400C5, 0x045A9B, 0x05AD5D,
 0x06A195, 0x06E6B2, 0x06EA66, 0x0800C4, 0x087140, 0x08B49A, 0x0B5A5C, 0x0D4294,
 0x0DBB1A, 0x0DCCB1, 0x0DD465, 0x0F6D85, 0x1000C3, 0x1021BF, 0x10E23F, 0x116899,
 0x116D01, 0x118544, 0x1231BB, 0x16B45B, 0x183B0C, 0x1A8493, 0x1B7619, 0x1B98B0,
 0x1BA7A3, 0x1BA864, 0x1DADA9, 0x1E013B, 0x1EDA84, 0x2000C2, 0x2042BE, 0x21C43E,
 0x22D098, 0x22DA00, 0x230A43, 0x2462BA, 0x27454C, 0x29FF7C, 0x2A0949, 0x2BBF51,
 0x2D685A, 0x305D2B, 0x30760B, 0x3331B7, 0x350892, 0x35B327, 0x36EC18, 0x370307,
 0x3730AF, 0x374EA2, 0x375063, 0x377B8B, 0x390D72, 0x3B5AA8, 0x3C023A, 0x3DB483,
 0x4000C1, 0x405303, 0x406946, 0x4084BD, 0x4363AB, 0x43883D, 0x44839E, 0x44D569,
 0x45635F, 0x45A097, 0x461442, 0x47D387, 0x481F2D, 0x48C4B9, 0x4A4B74, 0x4DD94E,
 0x4E8A4B, 0x51476D, 0x53FE7B, 0x541248, 0x553D6B, 0x577E50, 0x5AD059, 0x5EA314,
 0x5FD979, 0x60BA2A, 0x60E38E, 0x60EC0A, 0x650B0F, 0x6662B6, 0x67BB1D, 0x6A1091,
 0x6AAD12, 0x6AFB20, 0x6B5323, 0x6B6626, 0x6DD817, 0x6E0606, 0x6E60AE, 0x6E9CA1,
 0x6EA062, 0x6EF68A, 0x6F4577, 0x721A71, 0x76B4A7, 0x780439, 0x7B6157, 0x7B6882,
 0x8000C0, 0x80A602, 0x80D245, 0x8108BC, 0x840D0D, 0x85C3A4, 0x86C6AA, 0x87103C,
 0x89069D, 0x89A9B4, 0x89AA68, 0x8AC65E, 0x8B4096, 0x8C2841, 0x8ECD1B, 0x8FA686,
 0x903E2C, 0x9188B8, 0x92C928, 0x939108, 0x93AD8C, 0x949673, 0x9BB24D, 0x9CEF7D,
 0x9D144A, 0x9DCF52, 0xA0B36E, 0xA28E6C, 0xA74115, 0xA7FC7A, 0xA82447, 0xA83904,
 0xA9A1AC, 0xAA519F, 0xAA7A6A, 0xAAA160, 0xABF988, 0xAC1F2E, 0xAD3575, 0xAEFC4F,
 0xB5A058, 0xB8618F, 0xBA9510, 0xBBCD1E, 0xBD4613, 0xBD6D21, 0xBDB924, 0xBFB278,
 0xC17429, 0xC1C68D, 0xC1D809, 0xC6677E, 0xC6F753, 0xCA160E, 0xCAF1A5, 0xCCC4B5,
 0xCF761C, 0xD42090, 0xD55A11, 0xD5F61F, 0xD6A622, 0xD6CC25, 0xD8496F, 0xDBB016,
 0xDC0C05, 0xDCC0AD, 0xDD38A0, 0xDD4061, 0xDDEC89, 0xDE1F2F, 0xDE8A76, 0xE43470,
 0xE71F30, 0xEB237F, 0xEB6B54, 0xED68A6, 0xF00838, 0xF03137, 0xF04336, 0xF0A735,
 0xF16F34, 0xF2FF33, 0xF5DF32, 0xF6C256, 0xF6D081, 0xFB9F31, 0xFD8180, 0xFDA555 } ;

      uint16_t Bot=0;
      uint16_t Top=PacketBits;
      uint16_t MidSyndr=0;
      for( ; ; )
      { uint16_t Mid=(Bot+Top)>>1;
        MidSyndr = Syndrome[Mid]>>8;
        if(Syndr==MidSyndr) return (uint8_t)Syndrome[Mid];
        if(Mid==Bot) break;
        if(Syndr< MidSyndr) Top=Mid;
                       else Bot=Mid; }
      return 0xFF; }

} ;

#endif // __FLARM_H__
