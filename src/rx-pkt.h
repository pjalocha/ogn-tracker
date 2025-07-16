#include <stdint.h>
#include <string.h>

#include "bitcount.h"
#include "ogn.h"

const uint8_t Radio_SysID_FLR  = 0;  //
const uint8_t Radio_SysID_OGN  = 1;
const uint8_t Radio_SysID_ADSL = 2;
const uint8_t Radio_SysID_RID  = 3;
const uint8_t Radio_SysID_FNT  = 4;  // LoRa modulation
const uint8_t Radio_SysID_PAW  = 5;  // ADS-L over LDR
const uint8_t Radio_SysID_LDR  = 6;  // PilotAware over LDR
const uint8_t Radio_SysID_HDR  = 7;  // ADS-L HDR

// multi-system reception modes
const uint8_t Radio_SysID_FLR_ADSL  = 8; // FLARM with ADS-L
const uint8_t Radio_SysID_OGN_ADSL  = 9; // OGN with ADS-L

class FSK_RxPacket                    // Radio packet received by the RF chip
{ public:
   static const uint8_t MaxBytes=48;  // [bytes] max. number of bytes in the packet
   uint32_t Time;                     // [sec] UTC time slot
   uint16_t msTime;                   // [ms] reception time since the PPS[Time]
   union
   { uint16_t Flags;
     struct
     { uint8_t Channel;               // [ ] radio channel where the packet has been received
       uint8_t SysID   :6;            // [ ] 1=OGN, 2=ADS-L, ...
       bool Manchester :1;            // ADS-L and OGN are Manchester encoded on M-Band but not O-Band
       bool GoodCRC    :1;            // correct CRC has been detected
     } __attribute__((packed)) ;
   } __attribute__((packed)) ;
   uint8_t RSSI;                      // [-0.5dBm] receiver signal strength
    int8_t SNR;                       // [0.25dB]
    int8_t FreqErr;                   // [0.1kHz]
   uint8_t Bytes;                     // [bytes] actual packet size
   uint8_t Data[MaxBytes];            // decoded data bits/bytes (aligned to 32-bit)
   uint8_t Err [MaxBytes];            // Manchester decoding errors (for systems with Manchester encoding)

  public:

   static const char *SysName(uint8_t SysID)
   { static const char *Name[16] = { "FLR", "OGN", "ADL", "RID", "FNT", "PAW", "LDR", "HDR",
                                     "F+A", "O+A", "---", "---", "---", "---", "---", "---" } ;
     if(SysID<16) return Name[SysID];
     return 0; }

   // SYNC for various systems including multi-system reception with one SYNC
   static int SysSYNC(const uint8_t * &SYNC, uint8_t &PktLen, uint8_t SysID)
   { static const uint8_t SYNC_FLR6[10] = { 0x55, 0x99, 0xA5, 0xA9, 0x55, 0x66, 0x65, 0x96, 0x00, 0x00 };
     static const uint8_t SYNC_OGN1[10] = { 0xAA, 0x66, 0x55, 0xA5, 0x96, 0x99, 0x96, 0x5A, 0x00, 0x00 };
     static const uint8_t SYNC_ADSL[10] = { 0x55, 0x99, 0x95, 0xA6, 0x9A, 0x65, 0xA9, 0x6A, 0x00, 0x00 };
     static const uint8_t SYNC_LDR [10] = { 0xB4, 0x2B, 0x00, 0x00, 0x00, 0x00, 0x18, 0x71, 0x00, 0x00 };
     static const uint8_t SYNC_FLR_ADSL[4] = { 0x56, 0x66, 0x00, 0x00 } ;
     static const uint8_t SYNC_OGN_ADSL[4] = { 0x99, 0x95, 0x00, 0x00 } ;
     SYNC = 0; PktLen=0;
     if(SysID==Radio_SysID_FLR)      { SYNC=SYNC_FLR6;     PktLen=26;   return 8; }
     if(SysID==Radio_SysID_OGN)      { SYNC=SYNC_OGN1;     PktLen=26;   return 8; }
     if(SysID==Radio_SysID_ADSL)     { SYNC=SYNC_ADSL;     PktLen=24;   return 8; }
     if(SysID==Radio_SysID_PAW)      { SYNC=SYNC_LDR;      PktLen=31;   return 2; }
     if(SysID==Radio_SysID_LDR)      { SYNC=SYNC_LDR;      PktLen=31;   return 2; }
     if(SysID==Radio_SysID_FLR_ADSL) { SYNC=SYNC_FLR_ADSL; PktLen=26+3; return 2; }
     if(SysID==Radio_SysID_OGN_ADSL) { SYNC=SYNC_OGN_ADSL; PktLen=26+3; return 2; }
     return 0; }

   static int DiffBits(const uint8_t *Data, const uint8_t *Ref, const uint8_t *Mask, int Len)
   { int Count=0;
     for(int Idx=0; Idx<Len; Idx++)
     { Count+=Count1s((Data[Idx]^Ref[Idx])&Mask[Idx]); }
     return Count; }

   uint8_t DecodeSysID(void) // resolve multi-system receptions into unique types
   { if(SysID==Radio_SysID_OGN_ADSL)                       // if OGN+ADSL SYNC
     { const uint8_t SignADSL[3] = { 0x24, 0xB1, 0x80 } ;
       const uint8_t MaskADSL[3] = { 0xFF, 0xFF, 0xF0 } ;
       const uint8_t SignOGN [3] = { 0x9B, 0x2B, 0x60 } ;
       const uint8_t MaskOGN [3] = { 0xFF, 0xFF, 0xF8 } ;
            if(DiffBits(Data, SignADSL, MaskADSL, 3)<=1)
       { BitShift(20); Bytes=24; SysID=Radio_SysID_ADSL; }
       else if(DiffBits(Data, SignOGN, MaskOGN, 3)<=1)
       { BitShift(21); Bytes=26; SysID=Radio_SysID_OGN; }
       return SysID; }
     if(SysID==Radio_SysID_FLR_ADSL)                       // if FLARM+ADSL SYNC
     { const uint8_t SignADSL[3] = { 0xE4, 0x96, 0x30 } ;
       const uint8_t MaskADSL[3] = { 0xFF, 0xFF, 0xFE } ;
       const uint8_t SignFLR [3] = { 0x63, 0xF5, 0x6C } ;
       const uint8_t MaskFLR [3] = { 0xFF, 0xFF, 0xFE } ;
            if(DiffBits(Data, SignADSL, MaskADSL, 3)<=1)
       { BitShift(23); Bytes=24; SysID=Radio_SysID_ADSL; }
       else if(DiffBits(Data, SignFLR, MaskFLR, 3)<=1)
       { BitShift(23); Bytes=26; SysID=Radio_SysID_FLR; }
       return SysID; }
     return SysID; }

   // shift to the left a series of bytes by given number of bits
   static uint8_t BitShift(uint8_t *Data, uint8_t Bytes, uint8_t Shift)
   { if(Shift==0) return Bytes;                        // if nothing to shift then we are done
     uint8_t ByteOfs=Shift>>3; Shift&=7;               // split off the byte and bit part part
     if(Shift==0)                                      // if bit part is zero then simple move data
     { Bytes-=ByteOfs;                                 // there are now less bytes
       memmove(Data, Data+ByteOfs, Bytes);
       return Bytes; }                                 // return the number of bytes, which is same or lower
     uint8_t CmplShift=8-Shift;
     uint8_t Byte=Data[ByteOfs]<<Shift;                // take the first byte and shift it
     Bytes-=ByteOfs;
     for(uint8_t Idx=0; Idx<Bytes; Idx++)
     { uint8_t SrcByte=Data[ByteOfs+Idx+1];            // take the next byte
       Data[Idx] = Byte | (SrcByte>>CmplShift);        // (Byte1<<Shift) | (Byte2>>(8-Shift))
       Byte = SrcByte<<Shift; }
     return Bytes; }

   // shift Data and Err by given number of Bits
   void BitShift(int Bits)
   {      BitShift(Data, Bytes, Bits);
     Bytes=BitShift(Err, Bytes, Bits); }

   void Print(void (*CONS_UART_Write)(char), uint8_t WithData=0) const
   { // uint8_t ManchErr = Count1s(RxPktErr, 26);
     Format_String(CONS_UART_Write, "FSK_RxPacket: ");
     Format_HHMMSS(CONS_UART_Write, Time);
     CONS_UART_Write('+');
     Format_UnsDec(CONS_UART_Write, msTime, 4, 3);
     CONS_UART_Write(' '); Format_Hex(CONS_UART_Write, Channel);
     CONS_UART_Write('/');
     Format_SignDec(CONS_UART_Write, (int16_t)(-5*(int16_t)RSSI), 3, 1);
     Format_String(CONS_UART_Write, "dBm\n");
     if(WithData==0) return;
     for(uint8_t Idx=0; Idx<Bytes; Idx++)
     { CONS_UART_Write(' '); Format_Hex(CONS_UART_Write, Data[Idx]); }
     CONS_UART_Write('\r'); CONS_UART_Write('\n');
     for(uint8_t Idx=0; Idx<Bytes; Idx++)
     { CONS_UART_Write(' '); Format_Hex(CONS_UART_Write, Err[Idx]); }
     CONS_UART_Write('\r'); CONS_UART_Write('\n'); }

   int Print(char *Line, uint8_t WithData=0) const
   { int Len=0;
     // uint8_t ManchErr = Count1s(RxPktErr, 26);
     Len+=Format_String(Line+Len, "FSK_RxPacket: ");
     Len+=Format_HHMMSS(Line+Len, Time);
     Line[Len++]='+';
     Len+=Format_UnsDec(Line+Len, (uint32_t)msTime, 4, 3);
     Line[Len++]=' '; Len+=Format_Hex(Line+Len, Channel);
     Line[Len++]='/';
     Len+=Format_SignDec(Line+Len, (int32_t)(-5*(int16_t)RSSI), 3, 1);
     Len+=Format_String(Line+Len, "dBm\n");
     if(WithData==0) return Len;
     for(uint8_t Idx=0; Idx<Bytes; Idx++)
     { Line[Len++]=' '; Len+=Format_Hex(Line+Len, Data[Idx]); }
     Line[Len++]='\r'; Line[Len++]='\n';
     for(uint8_t Idx=0; Idx<Bytes; Idx++)
     { Line[Len++]=' '; Len+=Format_Hex(Line+Len, Err[Idx]); }
     Line[Len++]='\r'; Line[Len++]='\n';
     return Len; }

   bool NoErr(void) const
   { for(uint8_t Idx=0; Idx<Bytes; Idx++)
       if(Err[Idx]) return 0;
     return 1; }

   uint8_t ErrCount(void) const                         // count detected manchester errors
   { uint8_t Count=0;
     for(uint8_t Idx=0; Idx<Bytes; Idx++)
       Count+=Count1s(Err[Idx]);
     return Count; }

   uint8_t ErrCount(const uint8_t *Corr) const          // count errors compared to data corrected by FEC
   { uint8_t Count=0;
     for(uint8_t Idx=0; Idx<Bytes; Idx++)
       Count+=Count1s((uint8_t)((Data[Idx]^Corr[Idx])&(~Err[Idx])));
     return Count; }

 template <class OGNx_Packet>
  uint8_t Decode(OGN_RxPacket<OGNx_Packet> &Packet, LDPC_Decoder &Decoder, uint8_t Iter=32) const
  { uint8_t Check=0;
    uint8_t RxErr = ErrCount();                                // conunt Manchester decoding errors
    Decoder.Input(Data, Err);                                  // put data into the FEC decoder
    for( ; Iter; Iter--)                                       // more loops is more chance to recover the packet
    { Check=Decoder.ProcessChecks();                           // do an iteration
      if(Check==0) break; }                                    // if FEC all fine: break
    Decoder.Output(Packet.Packet.Byte());                      // get corrected bytes into the OGN packet
    RxErr += ErrCount(Packet.Packet.Byte());
    if(RxErr>15) RxErr=15;
    Packet.RxErr  = RxErr;
    Packet.RxChan = Channel;
    Packet.RxRSSI = RSSI;
    Packet.Correct= Check==0;
    return Check; }

} __attribute__((packed)) ;

