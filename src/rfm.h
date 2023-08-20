#pragma once

#include <stdint.h>

#include <Arduino.h>
#include <RadioLib.h>

#include "ogn.h"
#include "fanet.h"
#include "paw.h"
#include "fifo.h"
#include "manchester.h"
#include "timesync.h"
#include "freqplan.h"

const uint8_t SysID_OGN  = 1;
const uint8_t SysID_ADSL = 2;

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

class RFM_TRX: public FreqPlan,
#ifdef WITH_SX1276
   public SX1276
#endif
#ifdef WITH_SX1262
   public SX1262
#endif
{ public:
   // uint32_t BaseFrequency;            // [Hz] base frequency = channel #0
   // uint32_t ChannelSpacing;           // [Hz] spacing between channels
    int16_t FreqCorr;                 // [0.1ppm]
    int16_t Channel;                  // [integer] channel being used

    uint8_t SysID;                    // system being used: 1=OGN, 2=ADS-L

    uint8_t chipVer;                  // [] version ID read from the RF chip
     int8_t chipTemp;                 // [degC] temperature read from the RF chip

    uint8_t averRSSI;                 // [-0.5dBm]
    float   BkgRSSI;                  // [dBm]

    uint32_t Random;

    FIFO<Manch_RxPktData, 16> RxFIFO;

  public:

   RFM_TRX() :
#ifdef WITH_SX1276
    SX1276(new Module(Radio_PinCS, Radio_PinIRQ, Radio_PinRST, -1))
#endif
#ifdef WITH_SX1262
    SX1262(new Module(Radio_PinCS, Radio_PinIRQ, Radio_PinRST, Radio_PinBusy))
#endif
   { RxFIFO.Clear(); BkgRSSI=-100.0f; averRSSI=200; }

#ifdef WITH_SX1276
   int8_t getTemp(void) { return chipTemp=getTempRaw(); }
#endif

   int Init(void)
   { int State=0;
#ifdef WITH_SX1276
     State = beginFSK(868.2,          100.0,           50.0,        234.3,           14,              8);
     chipVer = getChipVersion();
     getTemp();
#endif
#ifdef WITH_SX1262
     State = beginFSK(868.2,          100.0,           50.0,        234.3,           14,              8,           1.6,         0);
     //               Freq[MHz], Bit-rate[kbps], Freq.dev.[kHz], RxBand.[kHz], TxPower[dBm], preamble[bits], TXCO volt.[V], use LDO[bool]
     State = setFrequency(868.2, 1); // calibrate
     // setTCXO(1.6);
     setDio2AsRfSwitch();
     chipVer=0x00;
#endif
     for(int Idx=0; Idx<4; Idx++)
       Random = (Random<<8) | randomByte();
     RxFIFO.Clear();
     setPlan(0);
     return State; }

// Errors:
//   0 => RADIOLIB_ERR_NONE
//  -1 => RADIOLIB_ERR_UNKNOWN
//  -2 => RADIOLIB_ERR_CHIP_NOT_FOUND
// -12 => RADIOLIB_ERR_INVALID_FREQUENCY
// -13 => RADIOLIB_ERR_INVALID_OUTPUT_POWER
// -20 => RADIOLIB_ERR_WRONG_MODEM

   int setManchFSK(uint32_t Time, uint8_t SubSlot, int8_t TxPower, uint8_t Sys)
   { SysID = Sys;
     Channel = getChannel(Time, SubSlot, Sys);
     uint8_t PktLen = getPktLen(Sys);    if(PktLen==0) return 0;
     const uint8_t *SYNC = SysSYNC(Sys); if(SYNC==0) return 0;
     ConfigManchFSK(PktLen, SYNC+1, 7);
     float Freq = 0.000001f*(BaseFreq + ChanSepar*Channel);
     if(FreqCorr!=0) Freq *= 1.0f+0.0000001*FreqCorr;
     setFrequency(Freq); // for SX1262 use 'false' as second argument to avoid mixer image calibration for SX1262
     setOutputPower(TxPower);
     setCurrentLimit(100);
     return 1; }

   static const uint8_t *SysSYNC(uint8_t Sys=1)
   { // OGNv1 SYNC:       0x0AF3656C encoded in Manchester
     static const uint8_t OGN1_SYNC[10] = { 0xAA, 0x66, 0x55, 0xA5, 0x96, 0x99, 0x96, 0x5A, 0x00, 0x00 };
     // const uint8_t *OGN_SYNC = OGN1_SYNC;
     // ADS-L SYNC:       0xF5724B18 encoded in Manchester (fixed packet length 0x18 is included)
     static const uint8_t ADSL_SYNC[10] = { 0x55, 0x99, 0x95, 0xA6, 0x9A, 0x65, 0xA9, 0x6A, 0x00, 0x00 };
     if(Sys==SysID_OGN ) return OGN1_SYNC;
     if(Sys==SysID_ADSL) return ADSL_SYNC;
     return 0; }

   int ConfigManchFSK(uint8_t PktLen, const uint8_t *SYNC, uint8_t SYNClen=8)         // Radio setup for OGN/ADS-L
   { int State=0;
     standby();
// #ifdef WITH_SX1276
//      State=config(RADIOLIB_SX127X_FSK_OOK);
// #endif
// #ifdef WITH_SX1262
//      State=config(RADIOLIB_SX126X_PACKET_TYPE_LORA);
// #endif
     State=setBitRate(100.0);                                    // [kpbs] 100kbps bit rate but we transmit Manchester encoded thus effectively 50 kbps
     State=setFrequencyDeviation(50.0);                          // [kHz]  +/-50kHz deviation
     State=setRxBandwidth(234.3);                                // [kHz]  250kHz bandwidth
     State=setEncoding(RADIOLIB_ENCODING_NRZ);
     State=setPreambleLength(8);                                 // [bits] minimal preamble
     State=setDataShaping(RADIOLIB_SHAPING_0_5);                 // [BT]   FSK modulation shaping
     State=setCRC(0, 0);                                         // disable CRC: we do it ourselves
     State=fixedPacketLengthMode(PktLen*2);                      // [bytes] Fixed packet size mode
     State=disableAddressFiltering();                            // don't want any of such features
#ifdef WITH_SX1276
     if(SYNC[0]==0x55)
       State = mod->SPIsetRegValue(RADIOLIB_SX127X_REG_SYNC_CONFIG, RADIOLIB_SX127X_PREAMBLE_POLARITY_55, 5, 5); // preamble polarity
     else if(SYNC[0]==0xAA)
       State = mod->SPIsetRegValue(RADIOLIB_SX127X_REG_SYNC_CONFIG, RADIOLIB_SX127X_PREAMBLE_POLARITY_AA, 5, 5); // preamble polarity
     State=setRSSIConfig(7, 0);                                  // set RSSI smoothing (3 bits) and offset (5 bits)
#endif
     State=setSyncWord((uint8_t *)SYNC, SYNClen);                // SYNC sequence: 8 bytes which is equivalent to 4 bytes before Manchester encoding
#ifdef WITH_SX1262
     State=setRxBoostedGainMode(true);                           // 2mA more current but boosts sensitivity
#endif
     return State; }

   uint8_t TxPacket[64];                              // Manchester-encoded packet just before transmission
   uint8_t RxPacket[64];                              // Manchester-encoded packet just after reception

   static uint8_t getPktLen(uint8_t SysID)
   { if(SysID==SysID_OGN ) return 26;
     if(SysID==SysID_ADSL) return 24;
     return 0; }
   uint8_t getPktLen(void) const { return getPktLen(SysID); }

   void TxManchFSK(const uint8_t *Packet)                       // transmit a packet using Manchester encoding
   { TxManchFSK(Packet, getPktLen()); }

   void TxManchFSK(const uint8_t *Packet, uint8_t Len)          // transmit a packet using Manchester encoding
   { // Serial.printf("TxManchFSK: %dB\n", Len);
     int TxLen=ManchEncode(TxPacket, Packet, Len);              // Manchester encode
     transmit(TxPacket, TxLen); }                               // transmit

   void ManchSlot(uint32_t msTimeLen, const uint8_t *TxPacket=0)
   { // Serial.printf("ManchSlot: %dms\n", msTimeLen);
     standby();
     uint32_t msStart = millis();
     startReceive();
     XorShift32(Random);
     if(TxPacket)
     { int TxTime = 20+Random%(msTimeLen-50);        // random transmission time
       ManchRx(TxTime);
       TxManchFSK(TxPacket);
       // Serial.printf("ManchSlot: startReceive()\n");
       startReceive();
       uint32_t msTime = millis()-msStart;
       if(msTime<msTimeLen) ManchRx(msTimeLen-msTime); }
     else ManchRx(msTimeLen); }

   int ManchRx(uint32_t msTimeLen)                                  // keep receiving packets for a given time [ms]
   { // Serial.printf("ManchRx: %dms\n", msTimeLen);
     int Count=0;
     uint32_t msStart = millis();                                   // [ms] start of the slot
     for( ; ; )
     { vTaskDelay(1);                                               // wait 1ms
       if(ManchRxPacket()>0) Count++;                               // check if a packet has been received
       uint32_t msTime = millis()-msStart;                          // [ms] time since start
       if(msTime>=msTimeLen) break; }                               // [ms] when reached the requesten time length then stop
#ifdef WITH_SX1262
     BkgRSSI += 0.05f*(getRSSI(false)-BkgRSSI);                      // [dBm] measure the noise level at the end of the slot and average
#endif
#ifdef WITH_SX1276
     BkgRSSI += 0.05f*(getRSSI(false, true)-BkgRSSI);             // [dBm] measure the noise level at the end of the slot and average
#endif
     averRSSI = floorf(-2.0f*BkgRSSI+0.5f);
     return Count; }

   bool readIRQ(void) { return digitalRead(mod->getIrq()); }

   int ManchRxPacket(void)
   { if(!readIRQ()) return 0;
     Manch_RxPktData *RxPkt = RxFIFO.getWrite();                // get place for a new packet in the queue
     int Ret=ManchRxPacket(RxPkt);
     if(Ret>0) RxFIFO.Write();
     return Ret; }

   int ManchRxPacket(Manch_RxPktData *RxPkt)                    // check if there is a new packet received
   { // if(!readIRQ()) return 0;                                   // use the IRQ line: not raised, then no received packet
     TickType_t msTime=0;
     TimeSync_Time(RxPkt->Time, msTime);                        // [sec, msec] timestamp
     RxPkt->msTime=msTime;     // posbly adjust ?
#ifdef WITH_SX1262
     uint32_t PktStat = getPacketStatus();                      // get RSSI
     Random += PktStat; XorShift32(Random);
     RxPkt->RSSI = PktStat;                                     // [-0.5dBm] average RSSI on the packet
     RxPkt->SNR  = 0; // PktStat>>8;                            // this should be SYNC RSSI but it does not fit this way
#endif
#ifdef WITH_SX1276
     RxPkt->RSSI = mod->SPIgetRegValue(RADIOLIB_SX127X_REG_RSSI_VALUE_FSK);
     Random += RxPkt->RSSI;
     RxPkt->SNR  = 0;
#endif
     uint8_t PktLen=getPktLen();
     readData(RxPacket, PktLen*2);                                             // read packet from the Radio
     uint8_t PktIdx=0;
     for(uint8_t Idx=0; Idx<PktLen; Idx++)                                     // loop over packet bytes
     { uint8_t ByteH = RxPacket[PktIdx++];
       ByteH = ManchesterDecode[ByteH]; uint8_t ErrH=ByteH>>4; ByteH&=0x0F; // decode Manchester, detect errors
       uint8_t ByteL = RxPacket[PktIdx++];
       ByteL = ManchesterDecode[ByteL]; uint8_t ErrL=ByteL>>4; ByteL&=0x0F; // second nibble
       RxPkt->Data[Idx]=(ByteH<<4) | ByteL;
       RxPkt->Err [Idx]=(ErrH <<4) | ErrL ; }
     RxPkt->Bytes   = PktLen;                                               // [bytes] actual packet size
     RxPkt->SysID   = SysID;                                                // Radio-system-ID
     RxPkt->Channel = Channel;                                              // Radio channel
     // Radio_RxCount[SysID]++;                                                // count packets
     // if(xSemaphoreTake(CONS_Mutex, 50))
     // { Serial.printf("RadioRx: %10d:%4d [%d:%d:%d] %+4.1fdBm %02X%02X%02X%02X\n",
     //                    RxPkt->Time, RxPkt->msTime, RxPkt->SysID, RxPkt->Channel, RxPkt->Bytes, -0.5*RxPkt->RSSI,
     //                    RxPkt->Data[3], RxPkt->Data[2], RxPkt->Data[1], RxPkt->Data[0] );
     //   xSemaphoreGive(CONS_Mutex); }
     return 1; }

   static int ManchEncode(uint8_t *Out, const uint8_t *Inp, uint8_t InpLen) // Encode packet bytes as Manchester
   { int Len=0;
     for(int Idx=0; Idx<InpLen; Idx++)                // loop over bytes and encode usinglookup table
     { uint8_t Byte=Inp[Idx];                         // data byte to be encoded
       Out[Len++]=ManchesterEncode[Byte>>4];          // use lookup table to encode upper nibble
       Out[Len++]=ManchesterEncode[Byte&0x0F]; }      // encode lower nibble
     return Len; }                                    // returns number of bytes in the encoded packet

} ;

