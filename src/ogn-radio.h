#pragma once

#include <stdint.h>

#include "main.h"

#include "format.h"
#include "ogn.h"
#include "ognconv.h"
#include "ogn1.h"
#include "ldpc.h"
#include "fifo.h"
#include "freqplan.h"
#include "rx-pkt.h"

// FLARMv6 SYNC: 0xF531FAB6 encoded in Manchester
// static const uint8_t FLR6_SYNC[10] = { 0x55, 0x99, 0xA5, 0xA9, 0x55, 0x66, 0x65, 0x96, 0x00, 0x00 };
// OGNv1 SYNC:       0x0AF3656C encoded in Manchester
// static const uint8_t OGN1_SYNC[10] = { 0xAA, 0x66, 0x55, 0xA5, 0x96, 0x99, 0x96, 0x5A, 0x00, 0x00 };
// static const uint8_t *OGN_SYNC = OGN1_SYNC;
// ADS-L SYNC:       0xF5724B18 encoded in Manchester (fixed packet length 0x18 is included)
// static const uint8_t ADSL_SYNC[10] = { 0x55, 0x99, 0x95, 0xA6, 0x9A, 0x65, 0xA9, 0x6A, 0x00, 0x00 };
// PilotAware SYNC, includes net-address which is always zero, and the packet size which is always 0x18 = 24
// static const uint8_t PAW_SYNC [10] = { 0xB4, 0x2B, 0x00, 0x00, 0x00, 0x00, 0x18, 0x71, 0x00, 0x00 };

const uint8_t Radio_SysID_FLR  = 0;  //
const uint8_t Radio_SysID_OGN  = 1;
const uint8_t Radio_SysID_ADSL = 2;
const uint8_t Radio_SysID_RID  = 3;
// const uint8_t Radio_SysID_PAW  = 3;  // GFSK but not Manchester encoded and different bitrate, deviation - not supported here yet
const uint8_t Radio_SysID_FNT  = 4;  // LoRa modulation

extern const char *Radio_SysName[5];

extern FIFO<OGN_TxPacket<OGN_Packet>, 4> OGN_TxFIFO;
extern FIFO<ADSL_Packet,              4> ADSL_TxFIFO;
// extern FIFO<ADSL_RID,                 4> RID_TxFIFO;
extern FIFO<FANET_Packet,             4> FNT_TxFIFO;

extern FIFO<Manch_RxPktData,         64> Manch_RxFIFO;
extern FIFO<FANET_RxPacket,           8> FNT_RxFIFO;

extern FreqPlan Radio_FreqPlan;       // RF frequency hopping scheme
extern QueueHandle_t Radio_SlotMsg;   // to tell the Radio_Task about the new time-slot

extern uint32_t Radio_TxCount[5];     // transmitted packet counters
extern uint32_t Radio_RxCount[5];     // received packet counters
extern  int32_t Radio_TxCredit;
extern float    Radio_BkgRSSI;        // [dBm] background noise seen by the receiver
extern float    Radio_PktRate;        // [Hz]

void Radio_Task(void *Parms);

#ifdef WITH_LORAWAN
#include "lorawan.h"
extern LoRaWANnode WANdev;
#endif
