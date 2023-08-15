#pragma once

#include <stdint.h>

#include <Arduino.h>
#include <RadioLib.h>

#include "ogn.h"
#include "fanet.h"
#include "paw.h"
#include "manchester.h"

class RFM_FSK_RxPktData             // OGN packet received by the RF chip
{ public:
   static const uint8_t Bytes=26;   // [bytes] number of bytes in the packet
   uint32_t Time;                   // [sec] Time slot
   uint16_t msTime;                 // [ms] reception time since the PPS[Time]
   uint8_t Channel;                 // [   ] channel where the packet has been recieved
   uint8_t RSSI;                    // [-0.5dBm] receiver signal strength
   uint8_t Data[Bytes];             // Manchester decoded data bits/bytes
   uint8_t Err [Bytes];             // Manchester decoding errors

  public:

   void Print(void (*CONS_UART_Write)(char), uint8_t WithData=0) const
   { // uint8_t ManchErr = Count1s(RxPktErr, 26);
     Format_String(CONS_UART_Write, "RxPktData: ");
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
     CONS_UART_Write('\r'); CONS_UART_Write('\n');
   }

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

} ;

class Manch_RxPktData                 // Manchester encoed packet received by the RF chip
{ public:
   static const uint8_t MaxBytes=26;  // [bytes] number of bytes in the packet
   uint32_t Time;                     // [sec] UTC time slot
   uint16_t msTime;                   // [ms] reception time since the PPS[Time]
   // uint32_t PosTime;                  // [ms] position timestamp in terms of the system time
   uint8_t Channel;                   // [   ] channel where the packet has been received
   uint8_t RSSI;                      // [-0.5dBm] receiver signal strength
    int8_t SNR;                       // [0.25dB]
    int8_t FreqErr;                   // [0.1kHz]
   uint8_t Bytes;                     // []
   uint8_t SysID;                     // [] 1=OGN, 2=ADS-L
   uint8_t Data[MaxBytes];            // Manchester decoded data bits/bytes
   uint8_t Err [MaxBytes];            // Manchester decoding errors

  public:

   void Print(void (*CONS_UART_Write)(char), uint8_t WithData=0) const
   { // uint8_t ManchErr = Count1s(RxPktErr, 26);
     Format_String(CONS_UART_Write, "RxPktData: ");
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
     CONS_UART_Write('\r'); CONS_UART_Write('\n');
   }

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

} ;

class RFM_TRX:
#ifdef WITH_SX1276
   public SX1276
#endif
#ifdef WITH_SX1262
   public SX1262
#endif
{ public:
   uint32_t BaseFrequency;            // [Hz] base frequency = channel #0
   uint32_t ChannelSpacing;           // [Hz] spacing between channels
    int16_t FreqCorr;                 // [0.1ppm]
    int16_t Channel;                  // [integer] channel being used

    uint8_t SysID;                    //
    uint8_t chipVer;                  // [] version ID read from the RF chip
     int8_t chipTemp;                 // [degC] temperature read from the RF chip
    uint8_t averRSSI;                 // [-0.5dB]

  public:

   RFM_TRX() :
#ifdef WITH_SX1276
    SX1276(new Module(LORA_CS, LORA_IRQ, LORA_RST, RADIOLIB_NC))
#endif
#ifdef WITH_SX1262
    SX1262(new Module(LORA_CS, LORA_IRQ, LORA_RST, LORA_BUSY))
#endif
   { }

   int Init(void)
   { int State=0;
#ifdef WITH_SX1276
     State = beginFSK(868.2,          100.0,           50.0,        234.3,            0,              8);
     // chipVer=0x00;
#endif
#ifdef WITH_SX1262
     State = beginFSK(868.2,          100.0,           50.0,        234.3,            0,              8,           1.6,         0);
     //               Freq[MHz], Bit-rate[kbps], Freq.dev.[kHz], RxBand.[kHz], TxPower[dBm], preamble[bits], TXCO volt.[V], use LDO[bool]
     State = setFrequency(868.2, 1); // calibrate
     // setTCXO(1.6);
     setDio2AsRfSwitch();
     chipVer=0x00;
#endif
     return State; }

   void setChannel(int Channel)
   { setFrequency(0.000001f*(1.0f+0.0000001*FreqCorr)*(BaseFrequency+ChannelSpacing*Channel)); }

// Errors:
//   0 => RADIOLIB_ERR_NONE
//  -1 => RADIOLIB_ERR_UNKNOWN
//  -2 => RADIOLIB_ERR_CHIP_NOT_FOUND
// -20 => RADIOLIB_ERR_WRONG_MODEM

   int ConfigManchFSK(uint8_t PktLen, const uint8_t *SYNC, uint8_t SYNClen=8)         // Radio setup for OGN/ADS-L
   { int State=0;
     State=setBitRate(100.0);                                    // [kpbs] 100kbps bit rate but we transmit Manchester encoded thus effectively 50 kbps
     State=setFrequencyDeviation(50.0);                          // [kHz]  +/-50kHz deviation
     State=setRxBandwidth(234.3);                                // [kHz]  250kHz bandwidth
     State=setPreambleLength(8);                                 // [bits] minimal preamble
     State=setDataShaping(RADIOLIB_SHAPING_0_5);                 // [BT]   FSK modulation shaping
     State=setCRC(0, 0);                                         // disable CRC: we do it ourselves
     State=fixedPacketLengthMode(PktLen*2);                      // [bytes] Fixed packet size mode
     State=disableAddressFiltering();                            // don't want any of such features
     State=setSyncWord((uint8_t *)SYNC, SYNClen);                // SYNC sequence: 8 bytes which is equivalent to 4 bytes before Manchester encoding
#ifdef WITH_SX1262
     State=setRxBoostedGainMode(true);                           // 2mA more current but boosts sensitivity
#endif
     return State; }

   static int ManchEncode(uint8_t *Out, const uint8_t *Inp, uint8_t InpLen) // Encode packet bytes as Manchester
   { int Len=0;
     for(int Idx=0; Idx<InpLen; Idx++)                // loop over bytes and encode usinglookup table
     { uint8_t Byte=Inp[Idx];                         // data byte to be encoded
       Out[Len++]=ManchesterEncode[Byte>>4];          // use lookup table to encode upper nibble
       Out[Len++]=ManchesterEncode[Byte&0x0F]; }      // encode lower nibble
     return Len; }                                    // returns number of bytes in the encoded packet

   uint8_t TxPacket[64];                              // Manchester-encoded packet just before transmission
   uint8_t RxPacket[64];                              // Manchester-encoded packet just after reception

   void TxManchFSK(const uint8_t *Packet, uint8_t Len)          // transmit a packet using Manchester encoding
   { int TxLen=ManchEncode(TxPacket, Packet, Len);              // Manchester encode
     transmit(TxPacket, TxLen); }                               // transmit


} ;

