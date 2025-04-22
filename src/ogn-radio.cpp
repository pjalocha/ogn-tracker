#include <Arduino.h>

#ifdef WITH_OGN

#include <RadioLib.h>
#include "ogn-radio.h"

#include "manchester.h"

#include "timesync.h"

#define WITH_FANET

// =======================================================================================================

 FreqPlan Radio_FreqPlan;       // RF frequency hopping scheme

 // quques for transmitted packets
 FIFO<OGN_TxPacket<OGN_Packet>, 4> OGN_TxFIFO;
 FIFO<ADSL_Packet,              4> ADSL_TxFIFO;
 // FIFO<ADSL_RID,                 4> RID_TxFIFO;
 FIFO<FANET_Packet,             4> FNT_TxFIFO;
 FIFO<PAW_Packet,               4> PAW_TxFIFO;

 // queues for received packets
 FIFO<FSK_RxPacket,            64> FSK_RxFIFO;
 FIFO<FANET_RxPacket,           8> FNT_RxFIFO;

 QueueHandle_t Radio_SlotMsg;   // to tell the Radio_Task about the new time-slot

// FLARMv6 SYNC: 0xF531FAB6 encoded in Manchester
static const uint8_t FLR6_SYNC[10] = { 0x55, 0x99, 0xA5, 0xA9, 0x55, 0x66, 0x65, 0x96, 0x00, 0x00 };
// OGNv1 SYNC:       0x0AF3656C encoded in Manchester
static const uint8_t OGN1_SYNC[10] = { 0xAA, 0x66, 0x55, 0xA5, 0x96, 0x99, 0x96, 0x5A, 0x00, 0x00 };
static const uint8_t *OGN_SYNC = OGN1_SYNC;
// ADS-L SYNC:       0xF5724B18 encoded in Manchester (fixed packet length 0x18 is included)
static const uint8_t ADSL_SYNC[10] = { 0x55, 0x99, 0x95, 0xA6, 0x9A, 0x65, 0xA9, 0x6A, 0x00, 0x00 };
// RID SYNC:         0xF5724B24 encoded in Manchester (fixed packet length 0x24 is included)
static const uint8_t RID_SYNC[10] = { 0x55, 0x99, 0x95, 0xA6, 0x9A, 0x65, 0xA6, 0x9A, 0x00, 0x00 };
// O-Band SYNC
// static const uint8_t OBAND_SYNC[10] = { 0xF5, 0x72, 0x4B, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 } ;
static const uint8_t OBAND_SYNC[10] = { 0xB4, 0x2B, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 } ;

// PilotAware SYNC, includes net-address which is always zero, and the packet size which is always 0x18 = 24
static const uint8_t PAW_SYNC [10] = { 0xB4, 0x2B, 0x00, 0x00, 0x00, 0x00, 0x18, 0x71, 0x00, 0x00 };

const char *Radio_SysName[6] = { "FLR", "OGN", "ADSL", "RID", "FNT", "PAW" } ;

static const uint8_t *Radio_SYNC[6] = { FLR6_SYNC, OGN_SYNC, ADSL_SYNC, RID_SYNC, 0, PAW_SYNC };
static const uint8_t  Radio_PktLen[6] = { 26, 26, 24, 36, 0, 26 };                          // [bytes] packet length, before Manchester encoding

uint32_t Radio_TxCount[6] = { 0, 0, 0, 0, 0, 0 } ; // transmitted packet counters
uint32_t Radio_RxCount[6] = { 0, 0, 0, 0, 0, 0 } ; // received packet counters

int32_t Radio_TxCredit = 60000;                        // [ms]
float   Radio_PktRate = 0.0f;                          // [Hz] received packet rate
const float Radio_PktUpdate = 0.05;                    // weight to update the packet rate
float   Radio_BkgRSSI = -105.0f;                       // [dBm] background noise seen by the receiver
const float Radio_BkgUpdate = 0.05;                    // weight to update the background noise

#ifdef TX_PA_GAIN
const float Radio_TxPwrGain = TX_PA_GAIN;
#else
const float Radio_TxPwrGain = 0;                       // [dBm] gain of a PA or loss of the filter/cable
#endif

// =======================================================================================================

#ifdef WITH_SX1276
static SX1276 Radio = new Module(Radio_PinCS, Radio_PinIRQ, Radio_PinRST, -1);                // create SX1276 RF module
static bool Radio_IRQ(void) { return digitalRead(Radio_PinIRQ); }
static const char *RF_ChipType = "SX1276";
#endif
#ifdef WITH_SX1262
static SX1262 Radio = new Module(Radio_PinCS, Radio_PinIRQ1, Radio_PinRST, Radio_PinBusy);    // create sx1262 RF module
static bool Radio_IRQ(void) { return digitalRead(Radio_PinIRQ1); }
static const char *RF_ChipType = "SX1262";
#endif

// =======================================================================================================
// Errors:
//   0 => RADIOLIB_ERR_NONE
//  -1 => RADIOLIB_ERR_UNKNOWN
//  -2 => RADIOLIB_ERR_CHIP_NOT_FOUND
// -20 => RADIOLIB_ERR_WRONG_MODEM

static int Radio_ConfigManchFSK(uint8_t PktLen, const uint8_t *SYNC, uint8_t SYNClen=8)         // Radio setup for FLR/OGN/ADS-L
{ int ErrState=0; int State=0;
  // uint32_t Time=millis();
  // Radio.standby();
  // vTaskDelay(1);
#ifdef WITH_SX1276
  if(Radio.getActiveModem()!=RADIOLIB_SX127X_FSK_OOK)
    State=Radio.setActiveModem(RADIOLIB_SX127X_FSK_OOK);
#endif
#ifdef WITH_SX1262
  if(Radio.getPacketType()!=RADIOLIB_SX126X_PACKET_TYPE_GFSK)
    State=Radio.config(RADIOLIB_SX126X_PACKET_TYPE_GFSK);
#endif
  if(State) ErrState=State;
  State=Radio.setBitRate(100.0);                                    // [kpbs] 100kbps bit rate but we transmit Manchester encoded thus effectively 50 kbps
  if(State) ErrState=State;
  State=Radio.setFrequencyDeviation(50.0);                          // [kHz]  +/-50kHz deviation
  if(State) ErrState=State;
#ifdef WITH_SX1262
  State=Radio.setRxBandwidth(234.3);                                // [kHz]  bandwidth - single side
  if(State) ErrState=State;
#endif
#ifdef WITH_SX1276
  State=Radio.setRxBandwidth(200.0);                                // [kHz]  bandwidth - single side
  if(State) ErrState=State;
  State=Radio.setAFCBandwidth(250.0);                               // [kHz]  auto-frequency-tune bandwidth
  if(State) ErrState=State;
  State=Radio.setAFC(1);                                            // enable AFC
  if(State) ErrState=State;
  State=Radio.setAFCAGCTrigger(RADIOLIB_SX127X_RX_TRIGGER_PREAMBLE_DETECT); //
  if(State) ErrState=State;
#endif
  State=Radio.setPreambleLength(16);                                // [bits] minimal preamble
  if(State) ErrState=State;
  State=Radio.setEncoding(RADIOLIB_ENCODING_NRZ);                   //
  if(State) ErrState=State;
  State=Radio.setDataShaping(RADIOLIB_SHAPING_0_5);                 // [BT]   FSK modulation shaping
  if(State) ErrState=State;
  State=Radio.setCRC(0, 0);                                         // disable CRC: we do it ourselves
  if(State) ErrState=State;
  State=Radio.fixedPacketLengthMode(PktLen*2);                      // [bytes] Fixed packet size mode
  if(State) ErrState=State;
#ifdef WITH_SX1276
  State=Radio.disableAddressFiltering();                            // don't want any of such features
  // we could actually use: invertPreamble(true) // true=0xAA, false=0x55
  if(SYNC[0]==0x55)
    State = Radio.mod->SPIsetRegValue(RADIOLIB_SX127X_REG_SYNC_CONFIG, RADIOLIB_SX127X_PREAMBLE_POLARITY_55, 5, 5); // preamble polarity
  else if(SYNC[0]==0xAA)
    State = Radio.mod->SPIsetRegValue(RADIOLIB_SX127X_REG_SYNC_CONFIG, RADIOLIB_SX127X_PREAMBLE_POLARITY_AA, 5, 5); // preamble polarity
  State=Radio.setRSSIConfig(7, 0);                                  // set RSSI smoothing (3 bits) and offset (5 bits)
  if(State) ErrState=State;
#endif
  State=Radio.setSyncWord((uint8_t *)SYNC, SYNClen);                // SYNC sequence: 8 bytes which is equivalent to 4 bytes before Manchester encoding
  if(State) ErrState=State;
#ifdef WITH_SX1262
  State=Radio.setRxBoostedGainMode(true);                           // 2mA more current but boosts sensitivity
  if(State) ErrState=State;
#endif
  // Time = millis()-Time;
  // Serial.printf("Radio_ConfigManchFSK(%d, ) (%d) %dms\n", PktLen, ErrState, Time);
  return ErrState; }                                                   // this call takes 18-19 ms

static int Radio_ConfigTxPower(float TxPower)      // set trannsmitter power
{ TxPower-=Radio_TxPwrGain;
  if(TxPower<0) TxPower=0;
#ifdef WITH_SX1276
  else if(TxPower>20) TxPower=20;
#endif
#ifdef WITH_SX1262
  else if(TxPower>22) TxPower=22;
#endif
  Radio.setOutputPower(TxPower);
  Radio.setCurrentLimit(140);                      // values are 0 to 140 mA for SX1262, default is 60
  return 0; }

static int ManchEncode(uint8_t *Out, const uint8_t *Inp, uint8_t InpLen) // Encode packet bytes as Manchester
{ int Len=0;
  for(int Idx=0; Idx<InpLen; Idx++)                // loop over bytes and encode usinglookup table
  { uint8_t Byte=Inp[Idx];                         // data byte to be encoded
    Out[Len++]=ManchesterEncode[Byte>>4];          // use lookup table to encode upper nibble
    Out[Len++]=ManchesterEncode[Byte&0x0F]; }      // encode lower nibble
  return Len; }                                    // returns number of bytes in the encoded packet

#ifdef WITH_SX1262
static int Radio_TxFSK(const uint8_t *Packet, uint8_t Len)
{ uint32_t usTxTime=Radio.getTimeOnAir(Len);                             // [usec]
  Radio_TxCredit-=usTxTime/1000;
  // uint32_t Time=millis();
  // LED_OGN_Blue();                                                     // 10ms flash for transmission
  int State=Radio.transmit((const uint8_t *)Packet, Len);                                 // transmit
  // LED_OGN_Off();
  // Time = millis()-Time;
  // Serial.printf("Radio_TxManchFSK(, %d=>%d) (%d) %dms\n", Len, TxLen, State, Time);  // for debug
  return State; }                                                        // this call takes 15-16 ms although the actuall packet transmission only 5-6 ms
#endif

#ifdef WITH_SX1276
static int Radio_TxFSK(const uint8_t *Packet, uint8_t Len)
{ // LED_OGN_Blue();
  // Radio.mod->SPIsetRegValue(RADIOLIB_SX127X_REG_PAYLOAD_LENGTH_FSK, Len);
  int State=Radio.startTransmit((const uint8_t *)Packet, Len);
  uint32_t usStart = micros();                                         // [usec] when transmission started
  uint32_t usTxTime=Radio.getTimeOnAir(Len);                           // [usec] predicted transmission time
   int32_t usLeft = usTxTime;                                          // [usec]
  Radio_TxCredit-=usTxTime/1000;
  for( ; ; )
  { uint32_t usTime = micros()-usStart;                                // [usec] time since transmission started
    usLeft = usTxTime-usTime;                                          // [usec] time left till the end of packet
    if(Radio_IRQ()) break;                                 // raised IRQ => end-of-data
    // uint16_t Flags=Radio.getIRQFlags(); if(Flags & RADIOLIB_SX127X_CLEAR_IRQ_FLAG_TX_DONE) break;
    if(usLeft>1500) { delay(1); continue; }
    if(usLeft<(-40)) break;
    taskYIELD(); }
  // State=Radio.finishTransmit();                         // adds a long delay and leaves a significant tail
  // Radio.clearIRQFlags();
  Radio.standby();
  Radio.clearIrqFlags(RADIOLIB_SX127X_FLAGS_ALL);
  // uint8_t RegPktLen = Radio.mod->SPIreadRegister(RADIOLIB_SX127X_REG_PAYLOAD_LENGTH_FSK);
  // uint8_t RegFixed = Radio.mod->SPIreadRegister(RADIOLIB_SX127X_REG_PACKET_CONFIG_1);
  // uint8_t RegDIO1 = Radio.mod->SPIreadRegister(RADIOLIB_SX127X_REG_DIO_MAPPING_1);
  // Serial.printf("Radio_TxFSK(, %d) usTxTime:%d, usLeft:%d [%d:%02X:%02X]\n", Len, usTxTime, usLeft, RegPktLen, RegFixed, RegDIO1);
  // LED_OGN_Off();
  return State; }
#endif

static uint8_t Radio_TxPacket[80];                 // Manchester-encoded packet just before transmission
static uint8_t Radio_RxPacket[80];                 // Manchester-encoded packet just after reception

static int Radio_TxManchFSK(const uint8_t *Packet, uint8_t Len)          // transmit a packet using Manchester encoding
{ int TxLen=ManchEncode(Radio_TxPacket, Packet, Len);                    // Manchester encode
  return Radio_TxFSK(Radio_TxPacket, TxLen); }

static int Radio_Receive(uint8_t PktLen, int Manch, uint8_t SysID, uint8_t Channel, TimeSync &TimeRef) // check if there is a new packet received
{ if(!Radio_IRQ()) return 0;                                             // use the IRQ line: not raised, then no received packet
  FSK_RxPacket *RxPkt = FSK_RxFIFO.getWrite();                           // get place for a new packet in the queue
#ifdef WITH_SX1262
  uint32_t PktStat = Radio.getPacketStatus();                            // get RSSI
  Random.RX += PktStat;
  RxPkt->RSSI = PktStat;                                                 // [-0.5dBm] average RSSI on the packet
  // float RSSI    = Radio.getRSSI();
  // RxPkt->RSSI = floorf(RSSI*(-2)+0.5);
  // Random.RX += RxPkt->RSSI;
#endif
#ifdef WITH_SX1276
  RxPkt->RSSI = Radio.mod->SPIgetRegValue(RADIOLIB_SX127X_REG_RSSI_VALUE_FSK);
  // float FreqErr = Radio.getAFCError();
  Random.RX += RxPkt->RSSI;
  // RxPkt->SNR  = 0;
#endif
  XorShift64(Random.Word);
  uint32_t msTime = millis();                                            // [ms] current system time
  // RxPkt->PosTime = TimeRef.sysTime;                                      // [ms] 
  RxPkt->msTime = msTime-TimeRef.sysTime;                                // [ms] time since the reference PPS
  RxPkt->Time = TimeRef.UTC;                                             // [sec] UTC PPS
  RxPkt->SNR  = 0; // PktStat>>8;                                        // this should be SYNC RSSI but it does not fit this way
  if(Manch)
  { Radio.readData(Radio_RxPacket, PktLen*2);                              // read packet from the Radio
    // Radio.startReceive();
    uint8_t PktIdx=0;
    for(uint8_t Idx=0; Idx<PktLen; Idx++)                                  // loop over packet bytes
    { uint8_t ByteH = Radio_RxPacket[PktIdx++];
      ByteH = ManchesterDecode[ByteH]; uint8_t ErrH=ByteH>>4; ByteH&=0x0F; // decode Manchester, detect errors
      uint8_t ByteL = Radio_RxPacket[PktIdx++];
      ByteL = ManchesterDecode[ByteL]; uint8_t ErrL=ByteL>>4; ByteL&=0x0F; // second nibble
      RxPkt->Data[Idx]=(ByteH<<4) | ByteL;
      RxPkt->Err [Idx]=(ErrH <<4) | ErrL ; }
  }
  else
  { Radio.readData(RxPkt->Data, PktLen);
    // Radio.startReceive();
    for(uint8_t Idx=0; Idx<PktLen; Idx++)
      RxPkt->Err[Idx]=0;
  }
  RxPkt->Manchester = Manch;
  RxPkt->Bytes   = PktLen;                                               // [bytes] actual packet size
  RxPkt->SysID   = SysID;                                                // Radio-system-ID
  RxPkt->Channel = Channel;                                              // Radio channel
  Radio_RxCount[SysID]++;                                                // count packets
#ifdef DEBUG_PRINT
  if(xSemaphoreTake(CONS_Mutex, 20))
  { Serial.printf("RadioRx: %5.3fs #%d [%d:%2d:%d] %+4.1fdBm\n",
             1e-3*millis(), Channel, SysID, PktLen, Manch, -0.5*RxPkt->RSSI);
    xSemaphoreGive(CONS_Mutex); }
#endif
  FSK_RxFIFO.Write();                                                  // complete the write into the queue of packets
  return 1; }

static int Radio_Receive(uint32_t msTimeLen, uint8_t PktLen, bool Manch, uint8_t SysID, uint8_t Channel, TimeSync &TimeRef) // keep receiving packets for a given time [ms]
{ uint32_t msStart = millis();                                     // [ms] start of the slot
  int PktCount=0;
  for( ; ; )
  { vTaskDelay(1);                                                 // wait 1ms
    PktCount+=Radio_Receive(PktLen, Manch, SysID, Channel, TimeRef);   // check if a packet has been received
    uint32_t msTime = millis()-msStart;                            // [ms] time since start
    if(msTime>=msTimeLen) break; }                                 // [ms] when reached the requesten time length then stop
#ifdef WITH_SX1262
  Radio_BkgRSSI += Radio_BkgUpdate*(Radio.getRSSI(false)-Radio_BkgRSSI);      // [dBm] measure the noise level at the end of the slot and average
#endif
#ifdef WITH_SX1276
  Radio_BkgRSSI += Radio_BkgUpdate*(Radio.getRSSI(false, true)-Radio_BkgRSSI);     // [dBm] measure the noise level at the end of the slot and average
#endif
  return PktCount; }                                               // return number of received packets

// TX/RX slot for a Manchester-encoded protocol
static int Radio_ManchSlot(uint8_t TxChannel, float TxPower, uint32_t msTimeLen, const uint8_t *TxPacket, uint8_t TxSysID,
                            uint8_t RxChannel, uint8_t RxSysID, TimeSync &TimeRef)
{ const uint8_t *TxSYNC = Radio_SYNC[TxSysID];
  const uint8_t *RxSYNC = Radio_SYNC[RxSysID];
  float TxFreq = 1e-6*Radio_FreqPlan.getChanFrequency(TxChannel);
  float RxFreq = 1e-6*Radio_FreqPlan.getChanFrequency(RxChannel);
  uint8_t TxPktLen = Radio_PktLen[TxSysID];
  uint8_t RxPktLen = Radio_PktLen[RxSysID];
  // Serial.printf("Radio_ManchSlot(%dms, %s, Tx:%d, Rx:%d) %5.1f/%5.1fMHz %1.0fdBm\n",
  //          msTimeLen, TxPacket?"RX/TX":"RX/--", TxSysID, RxSysID, RxFreq, TxFreq, TxPower);
  int PktCount=0;
  uint32_t msStart = millis();
  Radio.standby();
  Radio_ConfigManchFSK(RxPktLen, RxSYNC, 8);         // configure for reception
  Radio.setFrequency(RxFreq);                        // set frequency
#ifdef WITH_SX1276
  // Radio.setAFC(0);                                            // enable AFC
#endif
  Radio.startReceive();                              // start receiving
  XorShift64(Random.Word);                           // randomize
  if(TxPacket)
  { int TxTime = 25+Random.RX%(msTimeLen-50);        // random transmission time
    PktCount+=Radio_Receive(TxTime, RxPktLen, 1, RxSysID, RxChannel, TimeRef); // keep receiving packets till transmission time
    Radio.standby();
    Radio_ConfigManchFSK(TxPktLen, TxSYNC, 8);       // configure for transmission
    Radio_ConfigTxPower(TxPower);
    Radio.setFrequency(TxFreq);                      // set frequency
    Radio_TxManchFSK(TxPacket, TxPktLen);            // transmit the packet
    Radio_TxCount[TxSysID]++;
    Radio.standby();
    Radio_ConfigManchFSK(RxPktLen, RxSYNC, 8);       // configure for receiving
    Radio.setFrequency(RxFreq);                      // set frequency
#ifdef WITH_SX1276
    // Radio.setAFC(0);                                            // enable AFC
#endif
    Radio.startReceive();                            // start receiving again
    uint32_t msTime = millis()-msStart;
    if(msTime<msTimeLen) PktCount+=Radio_Receive(msTimeLen-msTime, RxPktLen, 1, RxSysID, RxChannel, TimeRef); }
  else
  { uint32_t msTime = millis()-msStart;
    if(msTime<msTimeLen) PktCount+=Radio_Receive(msTimeLen-msTime, RxPktLen, 1, RxSysID, RxChannel, TimeRef); }
  Radio.standby(); return PktCount; }

// =======================================================================================================

// Radio setup for PilotAware: GFSK, 38.4kbps, +/-9.6kHz and ADS-L LDR
static int Radio_ConfigLDR(uint8_t PktLen=PAW_Packet::Size, const uint8_t *SYNC=PAW_SYNC, uint8_t SYNClen=1)
{ int ErrState=0; int State=0;
#ifdef WITH_SX1276
  if(Radio.getActiveModem()!=RADIOLIB_SX127X_FSK_OOK)
    State=Radio.setActiveModem(RADIOLIB_SX127X_FSK_OOK);
#endif
#ifdef WITH_SX1262
  if(Radio.getPacketType()!=RADIOLIB_SX126X_PACKET_TYPE_GFSK)
    State=Radio.config(RADIOLIB_SX126X_PACKET_TYPE_GFSK);
#endif
  if(State) ErrState=State;
  State=Radio.setBitRate(38.4);                                     // [kpbs] 38.4kbps bit rate
  if(State) ErrState=State;
  State=Radio.setFrequencyDeviation(9.6);                           // [kHz]  +/-9.6kHz deviation
  if(State) ErrState=State;
  State=Radio.setRxBandwidth(58.6);                                 // [kHz]  50kHz bandwidth
  if(State) ErrState=State;
#ifdef WITH_SX1276
  State=Radio.setAFCBandwidth(58.6);                                // [kHz]  auto-frequency-tune bandwidth
  if(State) ErrState=State;
  State=Radio.setAFC(1);                                            // enable AFC
  if(State) ErrState=State;
  State=Radio.setAFCAGCTrigger(RADIOLIB_SX127X_RX_TRIGGER_PREAMBLE_DETECT); //
  if(State) ErrState=State;
#endif
  State=Radio.setEncoding(RADIOLIB_ENCODING_NRZ);
  if(State) ErrState=State;
  State=Radio.setPreambleLength(48);                                // [bits] very long preamble for Pilot-Aware
  if(State) ErrState=State;
  State=Radio.setDataShaping(RADIOLIB_SHAPING_1_0);                 // [BT]   FSK modulation shaping
  if(State) ErrState=State;
  State=Radio.setCRC(0, 0);                                         // disable CRC: we do it ourselves
  if(State) ErrState=State;
  State=Radio.fixedPacketLengthMode(PktLen+8);                      // [bytes] Fixed packet size mode
  if(State) ErrState=State;
#ifdef WITH_SX1276
  State=Radio.disableAddressFiltering();                            // don't want any of such features
  State=Radio.invertPreamble(true);                                 // true=0xAA, false=0x55
  // State = Radio.mod->SPIsetRegValue(RADIOLIB_SX127X_REG_SYNC_CONFIG, RADIOLIB_SX127X_PREAMBLE_POLARITY_AA, 5, 5); // preamble polarity
  State=Radio.setRSSIConfig(7, 0);                                  // set RSSI smoothing (3 bits) and offset (5 bits)
  if(State) ErrState=State;
#endif
  State=Radio.setSyncWord((uint8_t *)SYNC, SYNClen);                // SYNC sequence: 2 bytes, the rest we have to do in software
  if(State) ErrState=State;
#ifdef WITH_SX1262
  State=Radio.setRxBoostedGainMode(true);                           // 2mA more current but boosts sensitivity
  if(State) ErrState=State;
#endif
  // Time = millis()-Time;
  // Serial.printf("Radio_ConfigManchFSK(%d, ) (%d) %dms\n", PktLen, ErrState, Time);
  return ErrState; }                                                // this call takes 18-19 ms

static int Radio_TxPAW(const PAW_Packet &Packet)                    // transmit a PilotAware packet
{ memcpy(Radio_TxPacket, PAW_SYNC+1, 7);                            // first copy the remaining 7 bytes of the pre-data part
  memcpy(Radio_TxPacket+7, Packet.Byte, Packet.Size);               // copy packet to the buffer (internal CRC is already set)
  Packet.Whiten(Radio_TxPacket+7, Packet.Size);                     // whiten
  Radio_TxPacket[7+Packet.Size] = Packet.CRC8(Radio_TxPacket+7, Packet.Size); // add external CRC
  Radio_TxCount[Radio_SysID_PAW]++;
  return Radio_TxFSK(Radio_TxPacket, 7+Packet.Size+1); }            // send the packet out

/*
static int Radio_TxPAW(const PAW_Packet &Packet)                    // transmit a PilotAware packet
{ memcpy(Radio_TxPacket, PAW_SYNC+2, 6);
  memcpy(Radio_TxPacket+6, Packet.Byte, Packet.Size);               // copy packet to the buffer (internal CRC is already set)
  // Serial.printf("TxPAW[6+%d] ", Packet.Size);
  // for(int Idx=0; Idx<Packet.Size+6; Idx++)
  // { Serial.printf("%02X", Radio_TxPacket[Idx]); }
  // Serial.printf("\n");
  Packet.Whiten(Radio_TxPacket+6, Packet.Size);                     // whiten
  Radio_TxPacket[Packet.Size+6] = Packet.CRC8(Radio_TxPacket+6, Packet.Size); // add external CRC
  return Radio_TxFSK(Radio_TxPacket, Packet.Size+1+6); }            // send the packet out
*/
// =======================================================================================================

static int Radio_ConfigHDR(const uint8_t *SYNC=OBAND_SYNC, uint8_t SYNClen=2) // Radio setup for O-band ADS-L HDR
{ int ErrState=0; int State=0;
#ifdef WITH_SX1276
  State=Radio.setActiveModem(RADIOLIB_SX127X_FSK_OOK);
#endif
#ifdef WITH_SX1262
  State=Radio.config(RADIOLIB_SX126X_PACKET_TYPE_GFSK);
#endif
  if(State) ErrState=State;
  State=Radio.setBitRate(200.0);                                    // [kpbs] 
  if(State) ErrState=State;
  State=Radio.setFrequencyDeviation(50.0);                          // [kHz]  +/-50kHz deviation
  if(State) ErrState=State;
  State=Radio.setRxBandwidth(234.3);                                // [kHz]  250kHz bandwidth
  if(State) ErrState=State;
  State=Radio.setEncoding(RADIOLIB_ENCODING_NRZ);
  if(State) ErrState=State;
#ifdef WITH_SX1276
  State=Radio.setPreambleLength(8);                                 // [bits] minimal preamble
#endif
#ifdef WITH_SX1262
  State=Radio.setPreambleLength(8);                                 // [bits] minimal preamble
#endif
  if(State) ErrState=State;
  State=Radio.setDataShaping(RADIOLIB_SHAPING_0_5);                 // [BT]   FSK modulation shaping
  if(State) ErrState=State;
  State=Radio.setCRC(0, 0);                                         // disable CRC: we do it ourselves
  if(State) ErrState=State;
  State=Radio.variablePacketLengthMode();                           // variable packet length mode
  if(State) ErrState=State;
  State=Radio.disableAddressFiltering();                            // don't want any of such features
#ifdef WITH_SX1276
  // we could actually use: invertPreamble(true) // true=0xAA, false=0x55
  if(SYNC[0]==0x55)
    State = Radio.mod->SPIsetRegValue(RADIOLIB_SX127X_REG_SYNC_CONFIG, RADIOLIB_SX127X_PREAMBLE_POLARITY_55, 5, 5); // preamble polar>
  else if(SYNC[0]==0xAA)
    State = Radio.mod->SPIsetRegValue(RADIOLIB_SX127X_REG_SYNC_CONFIG, RADIOLIB_SX127X_PREAMBLE_POLARITY_AA, 5, 5); // preamble polar>
  State=Radio.setRSSIConfig(7, 0);                                  // set RSSI smoothing (3 bits) and offset (5 bits)
  if(State) ErrState=State;
#endif
  State=Radio.setSyncWord((uint8_t *)SYNC, SYNClen);                // SYNC sequence: 8 bytes which is equivalent to 4 bytes before M>
  if(State) ErrState=State;
#ifdef WITH_SX1262
  State=Radio.setRxBoostedGainMode(true);                           // 2mA more current but boosts sensitivity
  if(State) ErrState=State;
#endif
  return ErrState; }                                                // this call takes 18-19 ms

static int Radio_TxOBAND(uint8_t *Packet, uint8_t Len)              // transmit a packet on the O-Band
{ return Radio_TxFSK(Packet, Len); }

// =======================================================================================================

#ifdef WITH_FANET

static int Radio_FANETrxPacket(TimeSync &TimeRef)                  // attemp to receive FANET packet
{ if(!Radio_IRQ()) return 0;
  uint32_t msTime = millis();                                      // [ms] current system time
  // LED_Flash(10);
  // LED_OGN_Flash(10);
  FANET_RxPacket *RxPkt = FNT_RxFIFO.getWrite();                   // get space in the queue for the new packet
  int PktLen    = Radio.getPacketLength();   // [bytes]            // packet size
  float RSSI    = Radio.getRSSI();           // [dBm]              // RSSI
  float SNR     = Radio.getSNR();            // [dB]               // SNR
  float FreqOfs = Radio.getFrequencyError(); // [Hz]               // freq. offset
  // Serial.printf("FANET: [%d] %3.1fdB %3.1fdBm %+4.1fkHz\n", PktLen, SNR, RSSI, 1e-3*FreqOfs);
  RxPkt->Flags  = 0;
  if(PktLen>RxPkt->MaxBytes) { PktLen=RxPkt->MaxBytes; RxPkt->badCRC=1; } // if ppacket size above what we expected
  if(Radio.readData(RxPkt->Byte, PktLen)!=RADIOLIB_ERR_NONE) RxPkt->badCRC=1; // get the packet from the Radio
  RxPkt->Len=PktLen;
  // Serial.printf("FNT Rx[%d] %06X %3.1fdB %3.1fdBm %+4.1fkHz %c\n",
  //          PktLen, RxPkt->getAddr(), SNR, RSSI, 1e-3*FreqOfs, RxPkt->badCRC?'-':'+');
  RxPkt->msTime  = msTime-TimeRef.sysTime;                         // [ms] time since the reference PPS
  RxPkt->sTime   = TimeRef.UTC;                                    // [sec] UTC PPS
  RxPkt->FreqOfs = floorf(0.1*FreqOfs+0.5);
  RxPkt->SNR     = floorf(SNR*4+0.5);
  RxPkt->RSSI    = floorf(RSSI+0.5);
  FNT_RxFIFO.Write();
  Radio_RxCount[Radio_SysID_FNT]++;
  return 1; }

static int Radio_RxFANET(uint32_t msTimeLen, TimeSync &TimeRef)    // FANET reception slot
{ uint32_t msStart = millis();                                     // [ms] start of the slot
  int PktCount=0;
  for( ; ; )
  { vTaskDelay(1);                                                 // wait 1ms
    PktCount+=Radio_FANETrxPacket(TimeRef);                        // check if a packet has been received
    uint32_t msTime = millis()-msStart;                            // [ms] time since start
    if(msTime>=msTimeLen) break; }                                 // [ms] when reached the requesten time length then stop
#ifdef WITH_SX1262
  Radio_BkgRSSI += Radio_BkgUpdate*(Radio.getRSSI(false)-Radio_BkgRSSI);      // [dBm] measure the noise level at the end of the slot and average
#endif
#ifdef WITH_SX1276
  Radio_BkgRSSI += Radio_BkgUpdate*(Radio.getRSSI(false, true)-Radio_BkgRSSI); // [dBm] measure the noise level at the end of the slot and average
#endif
  return PktCount; }                                               // return nuber of packets received

static void Radio_TxFANET(FANET_Packet &Packet)                    // transmit a FANET packet
{ // Serial.printf("FNT Tx[%d] %06X\n", Packet.Len, Packet.getAddr());
  Radio.transmit(Packet.Byte, Packet.Len); Packet.Done=1;          // not clear, if we should wait here for the transmission to complete ?
  uint32_t usTxTime=Radio.getTimeOnAir(Packet.Len);                // [usec]
  Radio_TxCredit-=usTxTime/1000;
  Radio_TxCount[Radio_SysID_FNT]++; }

static void Radio_ConfigFANET(uint8_t CRa=4)                       // setup Radio for FANET
{                                                                  // first swith to LoRa mode
#ifdef WITH_SX1262
  if(Radio.getPacketType()!=RADIOLIB_SX126X_PACKET_TYPE_LORA)
    Radio.config(RADIOLIB_SX126X_PACKET_TYPE_LORA);
  //          Spreadng Factor,   Bandwidth,               Coding Rate,  low-data-rate-optimize
  Radio.setModulationParams(7, RADIOLIB_SX126X_LORA_BW_250_0, 4+CRa, RADIOLIB_SX126X_LORA_LOW_DATA_RATE_OPTIMIZE_OFF);
  //          Preamble length, CRC-type,      Payload (max) size, Header-Type,                    Invert-IQ
  Radio.setPacketParams(5, RADIOLIB_SX126X_LORA_CRC_ON, 40, RADIOLIB_SX126X_LORA_HEADER_EXPLICIT, RADIOLIB_SX126X_LORA_IQ_STANDARD);
#endif
#ifdef WITH_SX1276
  if(Radio.getActiveModem()!=RADIOLIB_SX127X_LORA)
    Radio.setActiveModem(RADIOLIB_SX127X_LORA);
#endif
  Radio.explicitHeader();
  Radio.setBandwidth(250.0);
  Radio.setSpreadingFactor(7);
  Radio.setCodingRate(4+CRa);
  Radio.invertIQ(false);
#ifdef WITH_SX1262
  Radio.setSyncWord(0xF1, 0x44);
#endif
#ifdef WITH_SX1276
  Radio.setSyncWord(0xF1);
#endif
  Radio.setPreambleLength(5);
  Radio.setCRC(true); }

// reception slot with a possible transmission if TxPacket != NULL
static int Radio_FANETslot(float Freq, float TxPower, uint32_t msTimeLen, FANET_Packet *TxPacket, TimeSync &TimeRef)
{ // Serial.printf("FANET Slot: %6.3fMHz %dms %c\n", 1e-6*Freq, msTimeLen, TxPacket?'T':'r');
  uint32_t msStart = millis();                       // [ms]
  Radio.standby();
  Radio_ConfigFANET();                               // setup for FANET, includes switching from FSK to LoRa
  Radio.setFrequency(Freq);                          // set frequency
  Radio.startReceive();                              // start receiving
  XorShift64(Random.Word);                           // randomize
  int PktCount=0;
  if(TxPacket)
  { uint32_t TxTime = 5;
    if(msTimeLen>35) TxTime+=Random.RX%(msTimeLen-35);   // random transmission time
    PktCount+=Radio_RxFANET(TxTime, TimeRef);            // keep receiving till transmission time
    Radio.standby();
    Radio_ConfigTxPower(TxPower);
    // uint32_t msTxTime=millis();
    Radio_TxFANET(*TxPacket);                        // transmit the packet
    // msTxTime = millis()-msTxTime;
    // Serial.printf("FANET TX: %d [%d] ms\n", TxTime, msTxTime);
    uint32_t msTime = millis()-msStart;
    if(msTime<msTimeLen) PktCount+=Radio_RxFANET(msTimeLen-msTime, TimeRef); }
  else
  { uint32_t msTime = millis()-msStart;
    if(msTime<msTimeLen) PktCount+=Radio_RxFANET(msTimeLen-msTime, TimeRef); }
  Radio.standby();
  return PktCount; }                                 // return number of received packets

#endif // WITH_FANET

// =======================================================================================================

#ifdef WITH_LORAWAN

// #include "lorawan.h"

LoRaWANnode WANdev;

static void Radio_TxLoRaWAN(uint8_t *Packet, uint8_t PktLen)
{ // Serial.printf("WAN Tx[%d]\n", PktLen);
  Radio.transmit(Packet, PktLen); }

static int Radio_RxLoRaWAN(uint8_t *Packet, uint8_t MaxPktLen, uint32_t msTimeLen, float *RSSI=0, float *SNR=0, float *FreqOfs=0)
{ uint32_t msStart=millis();
  // Serial.printf("RxLoRaWAN(%dms)\n", msTimeLen);
  Radio.startReceive();                             // start receiving
  for( ; ; )
  { vTaskDelay(1);
    uint32_t msTime = millis()-msStart;             // [ms] time since start
    if(msTime>=msTimeLen) break;                    // [ms] when reached the requesten time length then stop
    if(Radio_IRQ()) break; }                        // break, when packet arrives
  if(!Radio_IRQ()) return 0;
  int PktLen    = Radio.getPacketLength();          // [bytes]
  // Serial.printf("RxLoRaWAN: [%d]\n", PktLen);
  if(PktLen<=0 || PktLen>MaxPktLen) return 0;
  if(RSSI)    *RSSI    = Radio.getRSSI();           // [dBm]
  if(SNR)     *SNR     = Radio.getSNR();            // [dB]
  if(FreqOfs) *FreqOfs = Radio.getFrequencyError(); // [Hz]
  Radio.readData(Packet, PktLen);
  return PktLen; }

static void Radio_ConfigLoRaWAN(uint8_t Chan, bool TX, float TxPower, uint8_t CRa=4)
{
#ifdef WITH_SX1262
  if(Radio.getPacketType()!=RADIOLIB_SX126X_PACKET_TYPE_LORA)
    Radio.config(RADIOLIB_SX126X_PACKET_TYPE_LORA);
  //          Spreadng Factor,   Bandwidth,               Coding Rate,  low-data-rate-optimize
  Radio.setModulationParams(7, RADIOLIB_SX126X_LORA_BW_125_0, 4+CRa, RADIOLIB_SX126X_LORA_LOW_DATA_RATE_OPTIMIZE_OFF);
  //          Preamble length, CRC-type,                                                Payload (max) size, Header-Type,
  Radio.setPacketParams(8, TX?RADIOLIB_SX126X_LORA_CRC_ON:RADIOLIB_SX126X_LORA_CRC_OFF, 64, RADIOLIB_SX126X_LORA_HEADER_EXPLICIT,
                           TX?RADIOLIB_SX126X_LORA_IQ_STANDARD:RADIOLIB_SX126X_LORA_IQ_INVERTED);  // InvertIQ
#endif
#ifdef WITH_SX1276
  if(Radio.getActiveModem()!=RADIOLIB_SX127X_LORA)
    Radio.setActiveModem(RADIOLIB_SX127X_LORA);
#endif
  Radio.explicitHeader();
  Radio.setBandwidth(125.0);
  Radio.setSpreadingFactor(7);
  Radio.setCodingRate(4+CRa);
  Radio.invertIQ(!TX);                         // uplink without I/Q inversion, downlink with inversion
#ifdef WITH_SX1262
  Radio.setSyncWord(0x34, 0x44);
#endif
#ifdef WITH_SX1276
  Radio.setSyncWord(0x34);
#endif
  Radio.setPreambleLength(8);
  Radio.setCRC(TX);                            // uplink with CRC, downlink without CRC

  const float BaseFreq = 867.1;
  const float ChanStep =   0.2;
  Radio.setFrequency(BaseFreq+ChanStep*Chan);          // set frequency
  if(TX) Radio_ConfigTxPower(TxPower);

}

#endif

// =======================================================================================================

template <class Type>
 void Swap(Type &A, Type &B) { Type C=A; A=B; B=C; }

void Radio_Task(void *Parms)
{

  Radio_FreqPlan.setPlan(Parameters.FreqPlan);

#ifdef WITH_LORAWAN
  WANdev.Reset(getUniqueID(), Parameters.AppKey);     // set default LoRaWAN config.
  if(WANdev.ReadFromNVS()!=ESP_OK)                    // if can't read the LoRaWAN setup from NVS
  { WANdev.WriteToNVS(); }                            // then store the default
  if(Parameters.hasAppKey())                          // if there is an AppKey in the Paramters
  { if(!Parameters.sameAppKey(WANdev.AppKey))         // if LoRaWAN key different from the one in Parameters
    { WANdev.Reset(getUniqueID(), Parameters.AppKey); // then reset LoRaWAN to this key
      WANdev.Enable=1;
      WANdev.ABP=0;
      WANdev.WriteToNVS();                            // and save LoRaWAN config. to NVS
      xSemaphoreTake(CONS_Mutex, portMAX_DELAY);
      Serial.printf("LoRaWAN OTAA (re)set\n");
      // Format_String(CONS_UART_Write, "LoRaWAN: AppKey <- ");
      // Format_HexBytes(CONS_UART_Write, Parameters.AppKey, 16);
      // Format_String(CONS_UART_Write, " => ");
      // Format_SignDec(CONS_UART_Write, Err);
      // Format_String(CONS_UART_Write, "\n");
      xSemaphoreGive(CONS_Mutex); }
    Parameters.clrAppKey();                           // clear the AppKey in the Parameters and save it to Flash
    Parameters.WriteToNVS(); }
  else if(Parameters.hasAppSesKey() && Parameters.hasNetSesKey() && Parameters.DevAddr)
  { if(!Parameters.sameAppSesKey(WANdev.AppSesKey) || !Parameters.sameNetSesKey(WANdev.NetSesKey))
    { WANdev.Reset(getUniqueID());
      WANdev.Enable=1;
      WANdev.DevAddr=Parameters.DevAddr;
      memcpy(WANdev.AppSesKey, Parameters.AppSesKey, 16);
      memcpy(WANdev.NetSesKey, Parameters.NetSesKey, 16);
      WANdev.RxDelay = 5;
      WANdev.State   = 2;
      WANdev.UpCount = 0;
      WANdev.DnCount = 0xFFFFFFFF;
      WANdev.TxOptLen = 0;
      WANdev.ABP=1;
      WANdev.WriteToNVS();                            // and save LoRaWAN config. to NVS
      xSemaphoreTake(CONS_Mutex, portMAX_DELAY);
      Serial.printf("LoRaWAN ABP (re)set\n");
      xSemaphoreGive(CONS_Mutex); }
    Parameters.clrAppSesKey(); Parameters.clrNetSesKey();
    Parameters.WriteToNVS(); }
#endif

  SPI.begin(Radio_PinSCK, Radio_PinMISO, Radio_PinMOSI);  // CLK, MISO, MOSI, CS given by the Radio contructor
#ifdef Radio_SckFreq
  SPI.setFrequency(Radio_SckFreq);
#endif

#ifdef WITH_SX1276
  int State = Radio.beginFSK(868.2,          100.0,           50.0,        234.3,           14,              8);
  uint8_t chipVer = Radio.getChipVersion();
  if(State==RADIOLIB_ERR_NONE && chipVer==0x12) HardwareStatus.Radio=1;
                          // else LED_OGN_Red();
  // uint8_t chipVer = getChipVersion();
  // getTemp();
#endif
#ifdef WITH_SX1262
  int State = Radio.beginFSK(868.2,          100.0,           50.0,        234.3,            0,              8,           1.6,         0);
  //                     Freq[MHz], Bit-rate[kbps], Freq.dev.[kHz], RxBand.[kHz], TxPower[dBm], preamble[bits], TXCO volt.[V], use LDO[bool]
  // Serial.printf("Radio.begin() => %d\n", State);
  if(State==RADIOLIB_ERR_NONE) HardwareStatus.Radio=1;
                          // else LED_OGN_Red();
  State = Radio.setFrequency(1e-6*Radio_FreqPlan.BaseFreq, 1); // calibrate
  Radio.setTCXO(1.6);
  Radio.setDio2AsRfSwitch();
  // Radio.setDio1Action(IRQcall);
#endif

  TimeSync &TimeRef = GPS_TimeSync;
  char Line[120];

  int Len=sprintf(Line, "RF chip %s%s detected", RF_ChipType, HardwareStatus.Radio?"":" NOT");
  if(xSemaphoreTake(CONS_Mutex, 20))
  { Serial.println(Line);
    xSemaphoreGive(CONS_Mutex); }

  for( ; ; )
  { int PktCount=0;
    // char Line[120];

    // xQueueReceive(Radio_SlotMsg, &TimeRef, 2000);              // wait for "new time slot" from the GPS

    // uint32_t msTime = millis()-TimeRef.sysTime;                // [ms] time since PPS
    uint32_t msTime = TimeRef.getFracTime(millis());
    uint32_t Wait = 400-msTime;
    // Serial.printf("Pre-slot: %d [sec]  %d Wait:%d [ms]\n", TimeRef.UTC, msTime, Wait);
    if(Wait>300) Wait=300;

    // msTime = millis()-TimeRef.sysTime;
    // msTime = TimeRef.getFracTime(millis());

#ifdef WITH_FANET
    uint32_t FreqFNT = Radio_FreqPlan.getFreqFANET();           // frequency to transmit FANET
    if(FreqFNT)
    { Radio_ConfigFANET();
      Radio.setFrequency(1e-6*FreqFNT);
      Radio.startReceive();                                      // start receiving FANET
      for( ; ; )
      { PktCount+=Radio_FANETrxPacket(TimeRef);                  // any packet received ?
        if(FNT_TxFIFO.Full()) break;                             // when FANET packet to transmit, then stop this loop
        msTime = TimeRef.getFracTime(millis());
        if(msTime>=400) break;
        vTaskDelay(1); }
      FANET_Packet *FNTpacket = FNT_TxFIFO.getRead();            // get the FANET packet to transmit
      if(FNTpacket) FNT_TxFIFO.Read();
      XorShift64(Random.Word);
      int32_t msSlot = 400-msTime;                               //
      // Serial.printf("Pre-slot: %d [sec]  %d Slot:%d [ms]\n", TimeRef.UTC, msTime, msSlot);
      if(msSlot>40) PktCount+=Radio_FANETslot(FreqFNT, Parameters.TxPower, msSlot, FNTpacket, TimeRef);
    }

    // if(msTime<350)
    // { uint32_t msSlot = 380-msTime;
    //   uint32_t Freq = Radio_FreqPlan.getFreqFNT(TimeRef.UTC);
    //   if(Freq) Radio_FANETslot(Freq, Parameters.TxPower, msSlot, FNTpacket, TimeRef);
    //   else vTaskDelay(msSlot); }
    // else
#else
    if(Wait>0) vTaskDelay(Wait);
#endif // WITH_FANET
//      if(msTime<380) vTaskDelay(380-msTime);

    // if(xSemaphoreTake(CONS_Mutex, 20))
    // { Serial.printf("Radio: %10d:%8d %4dms\n", TimeRef.UTC, TimeRef.sysTime, msTime);
    //   xSemaphoreGive(CONS_Mutex); }

#ifdef WITH_PAW
    const PAW_Packet *PawPacket = PAW_TxFIFO.getRead();
    if(PawPacket) PAW_TxFIFO.Read();
    uint32_t FreqPAW = Radio_FreqPlan.getFreqOBAND();
    if(PawPacket && FreqPAW)                         // if there is a packet to be transmitted and the frequency plan allows it
    { Radio.standby();
      int Ret=Radio_ConfigLDR();
      // Serial.printf("PAW: Freq:%7.3fMHz (%d)\n", 1e-6*Freq, Ret);
      Radio.setFrequency(1e-6*FreqPAW);
      Radio_ConfigTxPower(Parameters.TxPower+13); // we can transmit PAW with higher power
      Radio_TxPAW(*PawPacket); }
#endif
    const OGN_TxPacket<OGN_Packet> *OgnPacket1 = OGN_TxFIFO.getRead();   // 1st OGN packet (possibly NULL)
    if(OgnPacket1) OGN_TxFIFO.Read();
    const OGN_TxPacket<OGN_Packet> *OgnPacket2 = OGN_TxFIFO.getRead();   // 2nd OGN packet (possibly NULL)
    if(OgnPacket2) { OGN_TxFIFO.Read(); if(Random.RX&4) Swap(OgnPacket1, OgnPacket2); }  // randomly swap
              else { OgnPacket2=OgnPacket1; }                                            // or 2nd = 1st
    // if(OgnPacket1) Serial.printf("OGN-1 for TX\n");
    // if(OgnPacket2) Serial.printf("OGN-2 for TX\n");

    const ADSL_Packet *AdslPacket1 = ADSL_TxFIFO.getRead();              // 1st ADS-L packet (possibly empty)
    if(AdslPacket1) ADSL_TxFIFO.Read();
    const ADSL_Packet *AdslPacket2 = ADSL_TxFIFO.getRead();              // 2nd ADS-L packet (posisbly empty)
    if(AdslPacket2) { ADSL_TxFIFO.Read(); if(Random.RX&8) Swap(AdslPacket1, AdslPacket2); } // randomly swap
               else { AdslPacket2=AdslPacket1; }
    // if(AdslPacket1) Serial.printf("ADSL-1 for TX\n");
    // if(AdslPacket2) Serial.printf("ADSL-2 for TX\n");

    // const ADSL_RID *AdslRID = RID_TxFIFO.getRead();
    // if(AdslRID) RID_TxFIFO.Read();
    // if(AdslRID) Serial.printf("ADSL-RID for TX\n");
/*
    int RxSys = Random.RX%3;
    Radio_ManchSlot(Radio_FreqPlan.getChannel(TimeRef.UTC, 0, 1), Parameters.TxPower, 450, OgnPacket1?OgnPacket1->Byte():0, Radio_SysID_OGN,
                             Radio_FreqPlan.getChannel(TimeRef.UTC, 0, RxSys>0), RxSys, TimeRef);
    Radio_ManchSlot(Radio_FreqPlan.getChannel(TimeRef.UTC, 1, 1), Parameters.TxPower, 450, OgnPacket2?OgnPacket2->Byte():0, Radio_SysID_OGN,
                             Radio_FreqPlan.getChannel(TimeRef.UTC, 1, RxSys>0), RxSys, TimeRef);
*/

    bool EU = Radio_FreqPlan.Plan<=1;

    uint32_t Hash = TimeRef.UTC;
    XorShift32(Hash);
    Hash *= 48271;
    XorShift32(Hash);
    Hash *= 48271;
    bool Odd = Count1s(Hash)&1;
    XorShift32(Hash);
    Hash *= 48271;
    bool Oband = EU && (Count1s(Hash)&1);

     int8_t  OGN_TxPwr = Parameters.TxPower;
     uint8_t OGN_Chan  = Radio_FreqPlan.getChannel(TimeRef.UTC, 0, 1);
    const uint8_t *OGN_Pkt = OgnPacket1?OgnPacket1->Byte():0;
     int8_t  ADSL_TxPwr = OGN_TxPwr;
     uint8_t ADSL_Chan  = OGN_Chan;
    const uint8_t *ADSL_Pkt = AdslPacket1?&(AdslPacket1->Version):0;
    if(Oband)
    { OGN_TxPwr+=13;
      OGN_Chan=Radio_FreqPlan.Channels; }
// #ifdef WITH_SX1276
//     if(OGN_TxPwr>20) OGN_TxPwr=20;
// #endif
// #ifdef WITH_SX1262
//     if(OGN_TxPwr>22) OGN_TxPwr=22;
// #endif

    bool OGNonly = !EU || !Parameters.RxADSL;         // OGN-only when out of EU/Africa

    msTime = millis()-TimeRef.sysTime;                // [ms] time since PPS
    uint32_t SlotLen = 800-msTime;
         if(SlotLen<200) SlotLen=200;
    else if(SlotLen>480) SlotLen=480;
    // Serial.printf("%5.3fs Slot #0: %dms\n", 1e-3*millis(), SlotLen);
    // the first time-slot
    if(!Odd || OGNonly)
             // Send OGN packet (if there) and receive OGN packets
             PktCount+=Radio_ManchSlot(OGN_Chan, OGN_TxPwr, SlotLen, OGN_Pkt, Radio_SysID_OGN,
                                       OGN_Chan, Radio_SysID_OGN, TimeRef);
             // Send ADS-L packet (if there) and receive ADS-L packets
        else PktCount+=Radio_ManchSlot(ADSL_Chan, ADSL_TxPwr, SlotLen, ADSL_Pkt, Radio_SysID_ADSL,
                                       ADSL_Chan, Radio_SysID_ADSL, TimeRef);

        // this is only to test ADS-L/RID transmissions
        // else Radio_ManchSlot(Radio_FreqPlan.getChannel(TimeRef.UTC, 0, 1), Parameters.TxPower, SlotLen, AdslRID?&(AdslRID->Version):0, Radio_SysID_RID,
        //                      Radio_FreqPlan.getChannel(TimeRef.UTC, 0, 1), Radio_SysID_ADSL, TimeRef);
/*
    if(FNTpacket)
    { uint32_t Freq = Radio_FreqPlan.getFreqFNT(TimeRef.UTC);
      if(Freq)
      { Radio.standby();
        Radio_ConfigFANET();
        Radio.setFrequency(Freq);
        Radio.setOutputPower(Parameters.TxPower);
        Radio_TxFANET(*FNTpacket); }
    }
*/

    msTime = millis()-TimeRef.sysTime;                // [ms] time since PPS
    SlotLen = 1200-msTime;
#ifdef WITH_LORAWAN
    static uint8_t WAN_RxPacket[64];                  //
    static uint32_t WAN_RespTick=0;                   // [msec]
    static uint8_t  WAN_BackOff=60;                   // [sec]
    bool WANtx = 0;
    if(WAN_BackOff) WAN_BackOff--;
    else if(WANdev.Enable && Parameters.TxWAN && Radio_FreqPlan.Plan<=1) // decide to transmit in this slot
    { if(WANdev.State==0 || WANdev.State==2) WANtx=1; } //
    if(WANtx) SlotLen = 1150-msTime;                    // if decision to transmit then stop the time slot a bit earlier
    else if(WANdev.State==1 || WANdev.State==3)         // if waiting for a reply
    { int32_t RespLeft = WAN_RespTick-millis();         // and the reply time getting close
      if(RespLeft>0 && RespLeft<1000) SlotLen=RespLeft-40; } // then adjust the time slot to be there in time
#endif

     OGN_Chan  = Radio_FreqPlan.getChannel(TimeRef.UTC, 1, 1);
     OGN_Pkt = OgnPacket2?OgnPacket2->Byte():0;
    ADSL_Chan  = OGN_Chan;
    ADSL_Pkt = AdslPacket2?&(AdslPacket2->Version):0;
    if(Oband)
    { OGN_Chan=Radio_FreqPlan.Channels; }

         if(SlotLen<200) SlotLen=200;
    else if(SlotLen>480) SlotLen=480;
    // Serial.printf("%5.3fs Slot #1: %dms\n", 1e-3*millis(), SlotLen);
    // the second time-slot
    if( Odd || OGNonly)
             // Send OGN packet (if there) and receive OGN packets
             PktCount+=Radio_ManchSlot(OGN_Chan, OGN_TxPwr, SlotLen, OGN_Pkt, Radio_SysID_OGN,
                                       OGN_Chan, Radio_SysID_OGN, TimeRef);
             // Send ADS-L packet (if there) and receive ADS-L packets
        else PktCount+=Radio_ManchSlot(ADSL_Chan, ADSL_TxPwr, SlotLen, ADSL_Pkt, Radio_SysID_ADSL,
                                       ADSL_Chan, Radio_SysID_ADSL, TimeRef);
        // this is only to test ADS-L/RID transmissions
        // else Radio_ManchSlot(Radio_FreqPlan.getChannel(TimeRef.UTC, 1, 1), Parameters.TxPower, SlotLen, AdslRID?&(AdslRID->Version):0, Radio_SysID_RID,
        //                      Radio_FreqPlan.getChannel(TimeRef.UTC, 1, 1), Radio_SysID_ADSL, TimeRef);
/*
             Radio_ManchSlot(Radio_FreqPlan.getChannel(TimeRef.UTC, 0, 1), Parameters.TxPower, 450, OgnPacket1?OgnPacket1->Byte():0, Radio_SysID_OGN,
                             Radio_FreqPlan.getChannel(TimeRef.UTC, 0, 1), Radio_SysID_OGN, TimeRef);
             Radio_ManchSlot(Radio_FreqPlan.getChannel(TimeRef.UTC, 1, 1), Parameters.TxPower, 450, OgnPacket2?OgnPacket2->Byte():0, Radio_SysID_OGN,
                             Radio_FreqPlan.getChannel(TimeRef.UTC, 1, 1), Radio_SysID_OGN, TimeRef);
*/

#ifdef WITH_LORAWAN
    if(WANtx)
    { XorShift64(Random.Word);                                        // random
      WANdev.Chan = Random.RX&7;                                      // choose random channel
      Radio_ConfigLoRaWAN(WANdev.Chan, 1, Parameters.TxPower);        // setup for LoRaWAN on given channel
      int RespDelay=0;
      int TxPktLen=0;
      if(WANdev.State==0)                                             // if not joined yet
      { uint8_t *TxPacket; TxPktLen=WANdev.getJoinRequest(&TxPacket); // produce Join-Request packet
        Radio_TxLoRaWAN(TxPacket, TxPktLen); WANdev.TxCount++;
        RespDelay=5000;          // transmit join-request packet
        WAN_BackOff=50+(Random.Word%19); XorShift64(Random.Word);
      } else if(WANdev.State==2)                                      // if joined the network
      { const uint8_t *PktData=0;                                     // data to be be transmitted
             if(OgnPacket1) PktData=OgnPacket1->Byte();
        else if(OgnPacket2) PktData=OgnPacket2->Byte();
        if(PktData)
        { OGN1_Packet *OGN = (OGN1_Packet *)PktData; if(!OGN->Header.Encrypted) OGN->Dewhiten();
          uint8_t *TxPacket;
          bool Short = !OGN->Header.NonPos && !OGN->Header.Encrypted  // decide if send a short (without header) or long format
                     && OGN->Header.AddrType==3 && OGN->Header.Address==(uint32_t)(getUniqueAddress()&0x00FFFFFF);
          if(Short)
          { TxPktLen=WANdev.getDataPacket(&TxPacket, PktData+4, 16, 1, ((Random.RX>>16)&0xF)==0x8 ); }
          else
          { TxPktLen=WANdev.getDataPacket(&TxPacket, PktData, 20, 1, ((Random.RX>>16)&0xF)==0x8 ); }
          Radio_TxLoRaWAN(TxPacket, TxPktLen);
          RespDelay = WANdev.RxDelay&0x0F;
          if(RespDelay<1) RespDelay=1; RespDelay*=1000;
          WAN_BackOff=50+(Random.Word%19);
          XorShift64(Random.Word);
        }
      }
      if(RespDelay)
      { uint32_t Time=millis();
        WAN_RespTick=Time+RespDelay;
        // Serial.printf("%5.3fs WAN Tx[%d] => %5.3fs\n", 1e-3*Time, TxPktLen, 1e-3*(Time+RespDelay));
      }
    }

    bool WANrx=0;
    uint32_t Time=millis();
     int32_t RespLeft = WAN_RespTick-Time;
    if(WANdev.State==1 || WANdev.State==3)                      // if State indicates we are waiting for the response
    { if(RespLeft<=5) WANdev.State--;                           // if time below 5 ticks we have not enough time
      else if(RespLeft<200) { WANrx=1; }                        // if more than 200ms then we can't wait this long now
      // if(WANrx==0 && RespLeft<=10) Serial.printf("%5.3fs WAN Rx missed %dms\n", 1e-3*Time, RespLeft);
    }
    if(WANrx)                                              // if reception expected from WAN
    { int RxLen=0;
      Radio_ConfigLoRaWAN(WANdev.Chan, 0, Parameters.TxPower);  // configure for reception
      uint32_t Time=millis();
      // Serial.printf("%5.3fs WAN Rx wait %dms\n", 1e-3*Time, WAN_RespTick-Time);
      // int msMaxTime=(WAN_RespTick-Time) + (Radio.getTimeOnAir(64)/1000) + 50;              // [ms] max-time to wait for the response
      int msMaxTime=(WAN_RespTick-Time) + 120;
      float RSSI=0; float SNR=0; float FreqOfs=0;
      RxLen=Radio_RxLoRaWAN(WAN_RxPacket, 64, msMaxTime, &RSSI, &SNR, &FreqOfs);
      // assume nothing received for now
      if(RxLen>0)
      { WANdev.RxCount++;
        WANdev.RxRSSI=RSSI; WANdev.RxSNR=SNR*4;
        uint32_t Time=millis();
        // Serial.printf("%5.3fs WAN Rx[%d]\n", 1e-3*Time, RxLen);
             if(WANdev.State==1) WANdev.procJoinAccept(WAN_RxPacket, RxLen);    // if join-request state then expect a join-accept packet
        else if(WANdev.State==3) RxLen=WANdev.procRxData(WAN_RxPacket, RxLen);  // if data send then respect ACK and/or downlink data packet
        WANdev.RxSilent=0; }
      else                                                 // if no packet received then retreat the State
      { WANdev.State--;
        WANdev.RxSilent++; if(WANdev.RxSilent>30) WANdev.Disconnect(); }
    }
    WANdev.WriteToNVS();                                 // store new WAN state in flash
#endif

    Radio_PktRate += Radio_PktUpdate*(PktCount-Radio_PktRate);
    Radio_TxCredit+= 10;                                // [ms]
    if(Radio_TxCredit>60000) Radio_TxCredit=60000;

    int LineLen=sprintf(Line, "Radio: Tx: %d:%d:%d:%d:%d:%d  Rx: %d:%d:%d:%d:%d:%d  %3.1fdBm %d pkts %3.1f pkt/s %3.1fs %d:%d",
             Radio_TxCount[0], Radio_TxCount[1], Radio_TxCount[2], Radio_TxCount[3], Radio_TxCount[4], Radio_TxCount[5],
             Radio_RxCount[0], Radio_RxCount[1], Radio_RxCount[2], Radio_RxCount[3], Radio_RxCount[4], Radio_RxCount[5],
             Radio_BkgRSSI, PktCount, Radio_PktRate, 0.001*Radio_TxCredit, Odd, Oband);
    SysLog_Line(Line, LineLen, 1, 25);
    if(xSemaphoreTake(CONS_Mutex, 20))
    { Serial.println(Line);
      xSemaphoreGive(CONS_Mutex); }

  }
}

// =======================================================================================================

#endif // WITH_OGN
