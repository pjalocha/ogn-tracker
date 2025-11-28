#pragma once

#include <stdio.h>
#include <stdint.h>
#include <string.h>

#include "aes128.h"

class MESHT_Packet
{ public:
   static const uint8_t MaxBytes = 64;
   static const uint8_t HeaderSize = 16; // first 16 bytes are header and then message
   union
   { uint8_t Byte[MaxBytes+2];
     struct
     { uint32_t Dst;
       uint32_t Src;
       uint32_t PktID;
       uint8_t PktFlags;
       uint8_t ChanHash;
       uint8_t NextHop;
       uint8_t RelayNode;
     } Header;
   } ;

   uint8_t Len;       // [bytes] packet length
   union
   { uint8_t Flags;
     struct
     { // uint8_t  CR:3;  // Coding rate used (RX) or to be used (TX)
       // bool hasCRC:1;  // CRC was there (RX)
       // bool badCRC:1;  // CRC was bad (RX)
       // bool   Done:1;
     } ;
   } ;

  public:
   MESHT_Packet() { Len=0; }

   uint8_t Dump(char *Out)
   { uint8_t Len=0;
     for(int Idx=0; Idx<this->Len; Idx++)
     { Len+=Format_Hex(Out+Len, Byte[Idx]); }
     return Len; }

   int setHeader(uint32_t Addr, uint8_t AddrType, uint8_t AcftType, uint32_t MAC, uint8_t MaxHops=4)
   { Header.Dst = 0xFFFFFFFF;
     if(Addr!=(MAC&0xFFFFFF)) return 0;
     Header.Src = MAC;
     Header.PktID = MAC;
     Byte[11] = AddrType | (AcftType<<2);
     if(MaxHops<1) MaxHops=1;
     if(MaxHops>7) MaxHops=7;
     Header.PktFlags = MaxHops | (MaxHops<<5);
     Header.ChanHash = 0x70;
     Header.NextHop = 0x00;
     Header.RelayNode = (uint8_t)MAC;
     return 1; }

         uint8_t *getMeshtHeader(void)       { return Byte; }               // pointer to the Meshtastic Header (16 bytes)
   const uint8_t *getMeshtHeader(void) const { return Byte; }
         uint8_t *getMeshtMsg   (void)       { return Byte+HeaderSize; }    // pointer to the Meshtastic Message
   const uint8_t *getMeshtMsg   (void) const { return Byte+HeaderSize; }
         uint8_t  getMeshtMsgLen(void) const { return Len-HeaderSize; }     // [bytes] size of the Message

   void getMeshtNonce(uint8_t  *Nonce, uint32_t Block=0) { getMeshtNonce((uint32_t *)Nonce, Block); }
   void getMeshtNonce(uint32_t *Nonce, uint32_t Block=0) const              // 128-bit key for AES128 ?
   { Nonce[0] = Header.PktID;
     Nonce[1] = 0;
     Nonce[2] = Header.Src;
     Nonce[3] = Block; }
   bool encryptMeshtMsg(AES128 &AES, uint8_t *Nonce)
   { if(Header.ChanHash!=0x70) return 0;          // we consider only ShortFast and open channel
     getMeshtNonce(Nonce);
     AES.EncryptMeshtantic(getMeshtMsg(), getMeshtMsgLen(), Nonce);
     return 1; }
   bool encryptMeshtMsg(AES128 &AES)
   { uint8_t Nonce[16];
     return encryptMeshtMsg(AES, Nonce); }

   int getMeshtRelayHops(void) const                 // how many hops the packet went through
   { uint8_t PktFlags = Header.PktFlags;
     uint32_t SrcAddr = Header.Src;
     uint8_t RelayNode = Header.RelayNode;
     uint8_t HopLimit = PktFlags&7;
     uint8_t HopStart = PktFlags>>5;
     if(HopStart>HopLimit) return HopStart-HopLimit;
     return RelayNode==(uint8_t)SrcAddr ? 0:-1; }

} ;
